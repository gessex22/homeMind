// A simulated robot gets new observations and a fresh belief on every pass.
// Geometry is consumed only by simulated physics and sensors, never by planning.
import { createMap, seedFootprint, gridOf, counts } from './mapping.js';
import { sense, SENSOR } from './sensor.js';
import { advance, STEP } from './physics.js';
import { solidos } from './world.js';
import { createOdometry, odomPose, readMotion, SENSORES_RUIDOSOS } from './odometry.js';
import { integrateFusedMotion, gateMotionCommand, motionFusionSummary } from './motion-fusion.js';
import { createMissionMapping, updateMissionMapping, missionSummary } from './mission-mapping.js';
import { createExplorer, startExplorer, exploreStep, setLeash } from './explorer.js';
import { mulberry32 } from './noise.js';

export const TRIAL_COUNT = 10;
export function trialOptions(index, baseSeed = 7) {
  return { id: index + 1, seed: (baseSeed + index * 7919) >>> 0, scanDirection: index % 2 ? -1 : 1 };
}
const terminal = new Set(['completado', 'limitado', 'bloqueado', 'refugio', 'localizacion_perdida']);
export function runTrial(scene, { id = 1, seed = 7, scanDirection = 1, maxSteps = 24000, leash = null, loopClosure = true, motionFusion = true } = {}) {
  const initial = { x: 0, z: 0, yaw: 0 };
  const map = createMap(scene.mapConfig), explorer = createExplorer();
  const belief = createOdometry(initial, { ...SENSORES_RUIDOSOS, seed });
  const sensor = { ...SENSOR, noise: .02, random: mulberry32((seed ^ 0x51f3) >>> 0) };
  const solids = solidos(scene);
  const mission = createMissionMapping(map, { loopClosure, motionFusion });
  let truth = { ...initial }, bump = false, steps = 0, collisions = 0, distance = 0, corrections = 0;
  explorer.scanDirection = scanDirection;
  seedFootprint(map, initial); setLeash(explorer, leash); startExplorer(explorer);
  for (; steps < maxSteps && !terminal.has(explorer.state); steps++) {
    let estimated = odomPose(belief);
    if (steps % 6 === 0) {
      const raw = sense(truth, scene.walls, sensor);
      const mapping = updateMissionMapping(mission, map, raw, estimated, explorer, steps * STEP, sensor, bump);
      Object.assign(belief, mapping.pose); estimated = odomPose(belief); corrections = mission.corrections;
    }
    const command = gateMotionCommand(mission.motion, exploreStep(explorer, map, estimated, STEP, bump));
    const next = advance(truth, command, STEP, solids);
    integrateFusedMotion(mission.motion, belief, readMotion(belief, next, truth, command, STEP, next.collision), STEP);
    distance += Math.hypot(next.x - truth.x, next.z - truth.z);
    collisions += Number(next.collision); bump = next.collision; truth = next;
  }
  const state = terminal.has(explorer.state) ? explorer.state : 'limite_simulacion';
  return { id, seed, direction: scanDirection === -1 ? 'barrido horario' : 'barrido antihorario', state, steps,
    seconds: steps * STEP, map, grid: { ...gridOf(map) },
    config: { odometry: { ...SENSORES_RUIDOSOS }, sensorNoise: .02, initial, leash, maxSteps, scanDirection, loopClosure, motionFusion },
    summary: { ...counts(map), corrections, frontiers: explorer.fronts.length,
      lostSeconds: explorer.localizacion.lostSeconds, slam: missionSummary(mission), motionFusion: motionFusionSummary(mission.motion),
      // Observer-only diagnostics, excluded from map alignment and commands.
      observer: { collisions, distance, positionError: Math.hypot(belief.x - truth.x, belief.z - truth.z) } },
  };
}
