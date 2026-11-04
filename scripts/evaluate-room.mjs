// Ground truth belongs to evaluation only. Never import this into the robot controller.
import { pathToFileURL } from 'node:url';
import { sense, SENSOR } from '../src/sensor.js';
import { SCENARIOS, solidos } from '../src/world.js';
import { createMap, integrate, seedFootprint, cell, gridOf, FREE, OCCUPIED, UNKNOWN } from '../src/mapping.js';
import { createExplorer, startExplorer, exploreStep, setLeash } from '../src/explorer.js';
import { advance, STEP, ROBOT_RADIUS } from '../src/physics.js';
import { createOdometry, readMotion, odomPose, SENSORES_RUIDOSOS } from '../src/odometry.js';
import { integrateFusedMotion, gateMotionCommand, motionFusionSummary } from '../src/motion-fusion.js';
import { mulberry32 } from '../src/noise.js';
import { createMissionMapping, updateMissionMapping, missionSummary } from '../src/mission-mapping.js';

const terminal = new Set(['completado', 'bloqueado', 'limitado', 'refugio', 'localizacion_perdida']);

function knownFootprint(map, pose) {
  const GRID = gridOf(map);
  const minCol = Math.floor((pose.x - ROBOT_RADIUS - GRID.originX) / GRID.resolution);
  const maxCol = Math.floor((pose.x + ROBOT_RADIUS - GRID.originX) / GRID.resolution);
  const minRow = Math.floor((pose.z - ROBOT_RADIUS - GRID.originZ) / GRID.resolution);
  const maxRow = Math.floor((pose.z + ROBOT_RADIUS - GRID.originZ) / GRID.resolution);
  for (let row = minRow; row <= maxRow; row++) {
    for (let col = minCol; col <= maxCol; col++) {
      const x = GRID.originX + col * GRID.resolution;
      const z = GRID.originZ + row * GRID.resolution;
      const nearX = Math.max(x, Math.min(x + GRID.resolution, pose.x));
      const nearZ = Math.max(z, Math.min(z + GRID.resolution, pose.z));
      if (Math.hypot(nearX - pose.x, nearZ - pose.z) >= ROBOT_RADIUS - 1e-8) continue;
      if (col < 0 || col >= GRID.width || row < 0 || row >= GRID.height) return false;
      if (map[row * GRID.width + col] !== FREE) return false;
    }
  }
  return true;
}

function distanceToWall(x, z, [x1, z1, x2, z2]) {
  const dx = x2 - x1, dz = z2 - z1;
  const t = Math.max(0, Math.min(1, ((x - x1) * dx + (z - z1) * dz) / (dx * dx + dz * dz)));
  return Math.hypot(x - x1 - t * dx, z - z1 - t * dz);
}

export function evaluateScenario(scenario, maxSteps = 24000, opts = {}) {
  const map = createMap(scenario.mapConfig);
  const GRID = gridOf(map);
  const explorer = createExplorer();
  const noisy = opts.odometry != null;
  const seed = opts.seed ?? 7;
  const belief = createOdometry({ x: 0, z: 0, yaw: 0 }, noisy ? { ...SENSORES_RUIDOSOS, ...opts.odometry, seed } : { seed });
  const sensorConfig = noisy ? { ...SENSOR, noise: opts.sensorNoise ?? .02, random: mulberry32((seed ^ 0x51f3) >>> 0) } : SENSOR;
  // Legacy baseline remains selectable for paired evaluations with equal seeds.
  const mission = createMissionMapping(map, { noisy, loopClosure: opts.loopClosure ?? false, motionFusion: opts.motionFusion ?? true, scanmatch: opts.scanmatch ?? {} });
  let pose = { x: 0, z: 0, yaw: 0, collision: false };
  let steps = 0, distance = 0, collisions = 0, unsafeSteps = 0, lastBump = false;
  let passedOpening = false, correcciones = 0, derivaCorregida = 0;
  const loopObserverEvents = [];
  const extent = { minX: 0, maxX: 0, minZ: 0, maxZ: 0 };
  seedFootprint(map, odomPose(belief));
  startExplorer(explorer);
  if (opts.leash) setLeash(explorer, opts.leash);

  for (; steps < maxSteps && !terminal.has(explorer.state); steps++) {
    let bp = odomPose(belief);
    if (steps % 6 === 0) {
      const rays = sense(pose, scenario.walls, sensorConfig);
      const mapping = updateMissionMapping(mission, map, rays, bp, explorer, steps * STEP, sensorConfig, lastBump);
      if (mapping.loop.rebuilt) {
        const before = mapping.loop.before.at(-1), after = mapping.pose;
        loopObserverEvents.push({ time: steps * STEP,
          errorBeforeM: Math.hypot(before.x - pose.x, before.z - pose.z), errorAfterM: Math.hypot(after.x - pose.x, after.z - pose.z),
          yawBefore: Math.abs(Math.atan2(Math.sin(before.yaw - pose.yaw), Math.cos(before.yaw - pose.yaw))),
          yawAfter: Math.abs(Math.atan2(Math.sin(after.yaw - pose.yaw), Math.cos(after.yaw - pose.yaw))) });
      }
      Object.assign(belief, mapping.pose); bp = odomPose(belief);
      correcciones = mission.corrections; derivaCorregida = mission.correctionDistance;
    }
    const command = gateMotionCommand(mission.motion, exploreStep(explorer, map, bp, STEP, lastBump));
    const prev = { x: pose.x, z: pose.z, yaw: pose.yaw };
    const next = advance(pose, command, STEP, solidos(scenario));
    integrateFusedMotion(mission.motion, belief, readMotion(belief, next, prev, command, STEP, next.collision), STEP);
    const bNext = odomPose(belief);
    const movement = Math.hypot(next.x - pose.x, next.z - pose.z);
    distance += movement;
    collisions += Number(Boolean(next.collision));
    if (movement > 0) {
      // Validate the swept body the ROBOT verified (its own belief)
      // against the map that existed BEFORE moving.
      const midpoint = { x: (bp.x + bNext.x) / 2, z: (bp.z + bNext.z) / 2 };
      if (!knownFootprint(map, bp) || !knownFootprint(map, midpoint) || !knownFootprint(map, bNext)) unsafeSteps++;
    }
    if (scenario.id !== 'empty' && next.z < -1.1) passedOpening = true;
    extent.minX = Math.min(extent.minX, next.x); extent.maxX = Math.max(extent.maxX, next.x);
    extent.minZ = Math.min(extent.minZ, next.z); extent.maxZ = Math.max(extent.maxZ, next.z);
    pose = next;
    lastBump = next.collision;
  }

  // Geometry is used only below to grade observations, never to choose actions.
  let floorKnown = 0, floorTotal = 0, hiddenUnknown = 0, observedBeyondInitialRange = 0;
  const { bounds } = scenario;
  const columns = Math.round((bounds.maxX - bounds.minX) / GRID.resolution);
  const rows = Math.round((bounds.maxZ - bounds.minZ) / GRID.resolution);
  for (let row = 0; row < rows; row++) {
    for (let col = 0; col < columns; col++) {
      const x = bounds.minX + (col + .5) * GRID.resolution, z = bounds.minZ + (row + .5) * GRID.resolution;
      // Exclude grid cells straddling a physical wall from the floor metric.
      if (scenario.walls.some(w => distanceToWall(x, z, w) < GRID.resolution * .75)) continue;
      floorTotal++;
      if (map[cell(x, z, map)] === FREE) {
        floorKnown++;
        if (Math.hypot(x, z) > SENSOR.range + .15) observedBeyondInitialRange++;
      }
      if (z < -1.1 && map[cell(x, z, map)] === UNKNOWN) hiddenUnknown++;
    }
  }

  // Sample each physical segment and accept either adjacent cell at its surface.
  let wallKnown = 0, wallTotal = 0;
  for (const [x1, z1, x2, z2] of scenario.walls) {
    const length = Math.hypot(x2 - x1, z2 - z1);
    const samples = Math.ceil(length / GRID.resolution);
    const nx = -(z2 - z1) / length, nz = (x2 - x1) / length;
    for (let i = 0; i < samples; i++) {
      const t = (i + .5) / samples;
      const x = x1 + (x2 - x1) * t, z = z1 + (z2 - z1) * t;
      // Surfaces inside a furniture footprint (its back against the wall,
      // the wall stretch it hides) can never be observed: not gradeable.
      if ((scenario.muebles ?? []).some(m => Math.abs(x - m.x) <= m.w / 2 + .06 && Math.abs(z - m.z) <= m.d / 2 + .06)) continue;
      wallTotal++;
      if ([-.025, .025].some(offset => map[cell(x + nx * offset, z + nz * offset, map)] === OCCUPIED)) wallKnown++;
    }
  }

  const stress = scenario.id === 'desorden';
  const safe = (stress || collisions === 0) && unsafeSteps === 0 && steps < maxSteps;
  const complete = explorer.state === 'completado' && explorer.fronts.length === 0
    && floorKnown / floorTotal >= .95 && wallKnown / wallTotal >= .95;
  const expected = scenario.id === 'narrow'
    ? explorer.state === 'limitado' && explorer.fronts.length > 0 && !passedOpening && hiddenUnknown > 0
    : stress ? terminal.has(explorer.state)
    : complete && (scenario.id === 'empty' || passedOpening);
  const scaleVerified = scenario.id !== 'large' || (observedBeyondInitialRange > 1000 && distance > 12);
  const end = odomPose(belief);
  const report = {
    escenario: scenario.id,
    evaluacion: scenario.label,
    condiciones: noisy ? 'Encoders y giróscopo con ruido y deriva; verdad reservada al evaluador' : 'Posición ideal y sensores sin ruido; verdad reservada al evaluador',
    odometria: noisy ? 'ruidosa' : 'exacta',
    slam: missionSummary(mission),
    fusion_movimiento: motionFusionSummary(mission.motion),
    observador_cierres: loopObserverEvents,
    error_odom_m: Number(Math.hypot(end.x - pose.x, end.z - pose.z).toFixed(4)),
    error_yaw_grados: Number((Math.abs(Math.atan2(Math.sin(end.yaw - pose.yaw), Math.cos(end.yaw - pose.yaw))) * 180 / Math.PI).toFixed(2)),
    aprobado: safe && expected && scaleVerified,
    estado: explorer.state,
    retorno: explorer.retorno ?? 'no',
    distancia_base_m: Number(Math.hypot(pose.x - (explorer.base?.x ?? 0), pose.z - (explorer.base?.z ?? 0)).toFixed(3)),
    tiempo_exploracion_s: Number(explorer.elapsed.toFixed(2)),
    tiempo_sin_localizacion_s: Number(explorer.localizacion.lostSeconds.toFixed(2)),
    max_tiempo_sin_localizacion_s: Number(explorer.localizacion.maxLostSeconds.toFixed(2)),
    localizacion_confirmada: explorer.localizacion.confirmed,
    correcciones_scan: correcciones,
    deriva_corregida_m: Number(derivaCorregida.toFixed(3)),
    pasos: steps,
    segundos_simulados: Number((steps * STEP).toFixed(2)),
    recorrido_metros: Number(distance.toFixed(3)),
    dimensiones_habitacion_m: [bounds.maxX - bounds.minX, bounds.maxZ - bounds.minZ],
    alcance_sensor_m: SENSOR.range,
    extension_recorrida_m: extent,
    celdas_observadas_fuera_del_alcance_inicial: observedBeyondInitialRange,
    barridos: explorer.scanned.length,
    suelo_observado_porcentaje: Number((floorKnown / floorTotal * 100).toFixed(2)),
    pared_observada_porcentaje: Number((wallKnown / wallTotal * 100).toFixed(2)),
    cruzo_abertura: scenario.id === 'empty' ? null : passedOpening,
    fronteras_pendientes: explorer.fronts.length,
    celdas_desconocidas_detras: hiddenUnknown,
    colisiones: collisions,
    pasos_con_huella_fuera_del_espacio_libre_conocido: unsafeSteps,
  };
  return { report, map, explorer, pose };
}

if (process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href) {
  const noisy = process.argv.includes('--ruido');
  const reports = Object.values(SCENARIOS).map(scene => evaluateScenario(scene, 24000, { ...(noisy ? { odometry: {}, seed: 7 } : {}), loopClosure: process.argv.includes('--slam') }).report);
  console.log(JSON.stringify(reports, null, 2));
  if (reports.some(report => !report.aprobado)) process.exitCode = 1;
}
