import test from 'node:test';
import assert from 'node:assert/strict';
import { compose, between, optimizeGraph } from '../src/pose-graph.js';
import { createSlam, observeSlam, matchSubmap, slamSummary, exportSlam } from '../src/slam.js';
import { createMap, integrate, seedFootprint, markBump, cell, OCCUPIED } from '../src/mapping.js';
import { sense, projectScan, SENSOR } from '../src/sensor.js';
import { mulberry32 } from '../src/noise.js';
import { evaluateScenario } from '../scripts/evaluate-room.mjs';
import { SCENARIOS } from '../src/world.js';
import { createLocalizationMonitor, tickLocalization, observeLocalization, invalidateLocalization } from '../src/localization.js';
import { createMissionMapping, updateMissionMapping } from '../src/mission-mapping.js';
import { createExplorer, exploreStep } from '../src/explorer.js';

const origin = { x: 0, z: 0, yaw: 0 };
const walls = [[-2, -2, 3, -2], [-2, -2, -2, 3], [3, -2, 3, 3], [-2, 3, 3, 3]];
const start = { x: 0, z: 0, yaw: Math.PI / 4 };

test('transformaciones SE2 respetan X/Z y el giro del robot', () => {
  const a = { x: 2, z: -1, yaw: Math.PI / 2 }, b = { x: -.2, z: .7, yaw: -2 };
  const recovered = compose(a, between(a, b));
  assert.ok(Math.hypot(recovered.x - b.x, recovered.z - b.z) < 1e-10);
  assert.ok(Math.abs(recovered.yaw - b.yaw) < 1e-10);
  const forward = compose({ ...origin, yaw: Math.PI / 2 }, { x: 0, z: -1, yaw: 0 });
  assert.ok(Math.abs(forward.x + 1) < 1e-10 && Math.abs(forward.z) < 1e-10);
});

test('el grafo distribuye un cierre en la trayectoria y conserva el ancla', () => {
  const nodes = [origin, { x: 1.1, z: 0, yaw: .05 }, { x: 1.2, z: 1, yaw: .1 }, { x: .3, z: 1, yaw: .1 }, { x: .4, z: .1, yaw: .12 }].map(p => ({ pose: { ...p } }));
  const edges = nodes.slice(1).map((n, i) => ({ from: i, to: i + 1, measurement: between(nodes[i].pose, n.pose), weight: 100, yawWeight: 400 }));
  edges.push({ from: 0, to: 4, measurement: origin, weight: 900, yawWeight: 1600 });
  const result = optimizeGraph(nodes, edges);
  assert.deepEqual(result.poses[0], origin);
  assert.ok(result.after < result.before * .1);
  assert.ok(Math.hypot(result.poses[4].x, result.poses[4].z) < .04);
  assert.ok(Math.hypot(result.poses[2].x - nodes[2].pose.x, result.poses[2].z - nodes[2].pose.z) > .05, 'También corrige poses históricas');
  assert.equal(nodes[4].pose.x, .4, 'La optimización es transaccional');
});

function observedReference() {
  const map = createMap(), slam = createSlam(map, start), sensor = { ...SENSOR, noise: .02, random: mulberry32(19) };
  for (let i = 0; i < 30; i++) {
    const raw = sense(start, walls, sensor), result = observeSlam(slam, map, raw, start, sensor, i * .1);
    if (!result.rebuilt) integrate(map, result.pose, projectScan(raw, result.pose));
  }
  return { map, slam, reference: slam.submaps[0] };
}
function scanPoints(raw) {
  return raw.flatMap((r, i) => r.kind === 'wall' ? [{ x: -Math.sin((i / (raw.length - 1) - .5) * SENSOR.fov) * r.distance, z: -Math.cos((i / (raw.length - 1) - .5) * SENSOR.fov) * r.distance }] : []);
}
test('una esquina observada reconoce la pose y no necesita ángulos verdaderos', () => {
  const { reference } = observedReference();
  const raw = sense(start, walls).map(r => ({ ...r, angle: 999 }));
  const match = matchSubmap(reference, scanPoints(raw), { x: .3, z: .15, yaw: .09 });
  assert.equal(match.accepted, true, JSON.stringify(match));
  assert.ok(Math.hypot(match.pose.x, match.pose.z) < .08);
  assert.ok(Math.abs(match.pose.yaw) < .06, 'Tolerancia de una celda de 10 cm sobre paredes a 2 m: ' + JSON.stringify(match));
});
test('pared uniforme, mapa desconocido y lecturas sin paredes no reconocen lugares', () => {
  const { reference } = observedReference();
  const line = Array.from({ length: 40 }, (_, i) => ({ x: i * .05 - 1, z: -2 }));
  assert.equal(matchSubmap(reference, line, origin).accepted, false);
  assert.equal(matchSubmap(reference, [], origin).accepted, false);
  const empty = { map: createMap(reference.map.grid), field: new Float32Array(reference.map.length) };
  assert.equal(matchSubmap(empty, scanPoints(sense(start, walls)), origin).accepted, false);
});
test('una coincidencia de extremos se rechaza si sus rayos cruzan una pared conocida', () => {
  const { reference } = observedReference();
  for (let i = 0; i < 121; i++) {
    const a = (i / 120 - .5) * SENSOR.fov;
    reference.map[cell(-Math.sin(a) * .8, -Math.cos(a) * .8, reference.map)] = OCCUPIED;
  }
  assert.equal(matchSubmap(reference, scanPoints(sense(start, walls)), origin).accepted, false);
});
test('observar sin salir del lugar no se cuenta como cierre por el paso del tiempo', () => {
  const { slam, map } = observedReference();
  for (let i = 30; i < 200; i++) observeSlam(slam, map, sense(start, walls), start, SENSOR, i * .1);
  assert.equal(slam.closures.length, 0);
  assert.equal(slam.candidates, 0);
});
test('un contacto pendiente tiene prioridad sobre un cierre y conserva el reflejo de choque', () => {
  const { slam, map } = observedReference(), mission = createMissionMapping(createMap());
  mission.slam = slam;
  const e = createExplorer(); e.state = 'navegando';
  slam.pending = { count: 2, time: 19.5, submap: slam.submaps[0].id, delta: origin };
  const result = updateMissionMapping(mission, map, sense(start, walls), start, e, 20, SENSOR, true);
  assert.equal(result.loop.rebuilt, false);
  assert.equal(e.state, 'navegando'); assert.equal(slam.pending, null);
  exploreStep(e, map, result.pose, 1 / 60, true);
  assert.equal(e.state, 'retroceso');
});

test('un regreso con deriva requiere confirmaciones y reconstruye historia y contactos', () => {
  const map = createMap(), slam = createSlam(map, start), sensor = { ...SENSOR, noise: .02, random: mulberry32(42) };
  seedFootprint(map, start);
  let time = 0, lastPose = start, event;
  function scan(truth, estimated) {
    const raw = sense(truth, walls, sensor);
    const references = slam.submaps.slice();
    const result = observeSlam(slam, map, raw, estimated, sensor, time);
    if (!result.rebuilt) integrate(map, result.pose, projectScan(raw, result.pose));
    else {
      event = { ...result, truth };
      for (const old of references) assert.notEqual(slam.submaps.find(m => m.id === old.id)?.map, old.map, 'Reconstruye también los submapas históricos');
    }
    lastPose = result.pose; time += .1;
  }
  for (let i = 0; i < 35; i++) scan(start, start);
  map[cell(1.75, 1.75, map)] = OCCUPIED;
  const corners = [[0, 0], [1.1, 0], [1.1, 1.1], [0, 1.1], [0, 0]];
  for (let leg = 0; leg < 4; leg++) for (let i = 1; i <= 50; i++) {
    const progress = (leg + i / 50) / 4;
    const truth = { x: corners[leg][0] + (corners[leg + 1][0] - corners[leg][0]) * i / 50, z: corners[leg][1] + (corners[leg + 1][1] - corners[leg][1]) * i / 50, yaw: start.yaw };
    const estimate = { x: truth.x + .35 * progress, z: truth.z + .15 * progress, yaw: truth.yaw + .1 * progress };
    scan(truth, estimate);
    if (leg === 1 && i === 20) markBump(map, estimate, false, .4);
  }
  for (let i = 0; i < 25 && !event; i++) scan(start, { x: .35, z: .15, yaw: start.yaw + .1 });
  assert.ok(event, JSON.stringify(slamSummary(slam)));
  assert.ok(Math.hypot(event.pose.x - event.truth.x, event.pose.z - event.truth.z) < .12, JSON.stringify(event.pose));
  assert.ok(event.event.confirmations >= 3);
  assert.ok(event.event.costAfter < event.event.costBefore);
  assert.ok(event.before.some((p, i) => Math.hypot(p.x - event.after[i].x, p.z - event.after[i].z) > .1));
  assert.notEqual(map[cell(1.75, 1.75, map)], OCCUPIED);
  assert.equal(slam.contacts.length, 1);
  const contact = slam.contacts[0], p = compose(slam.nodes[contact.anchor].pose, contact.local);
  const hit = cell(p.x - Math.sin(p.yaw) * contact.distance, p.z - Math.cos(p.yaw) * contact.distance, map);
  assert.equal(map[hit], OCCUPIED, 'El cierre conserva el obstáculo revelado por contacto');
  const exported = JSON.parse(JSON.stringify(exportSlam(slam)));
  assert.equal(exported.observations.length, slam.records.length);
  assert.ok(exported.observations.every(r => r.anchor < exported.nodes.length));
  assert.equal(exported.contacts.length, 1);
});

test('un cambio de marco invalida confirmación sin borrar un tiempo de pérdida existente', () => {
  const m = createLocalizationMonitor(); tickLocalization(m, .2);
  invalidateLocalization(m);
  assert.equal(m.confirmed, false); assert.equal(m.lastReliablePose, null); assert.equal(m.lostSeconds, 0);
  tickLocalization(m, .4); invalidateLocalization(m);
  assert.ok(Math.abs(m.lostSeconds - .4) < 1e-10);
  observeLocalization(m, { reliable: true }, origin);
  assert.equal(m.confirmed, false, 'Una observación aislada no confirma');
  for (let i = 0; i < 2; i++) { tickLocalization(m, .1); observeLocalization(m, { reliable: true }, origin); }
  assert.equal(m.confirmed, true); assert.equal(m.lostSeconds, 0);
  const lost = createLocalizationMonitor(); tickLocalization(lost, 4);
  const before = lost.lostSeconds; invalidateLocalization(lost); assert.equal(lost.lostSeconds, before);
});

test('el límite de memoria es explícito y no elimina la historia anterior', () => {
  const map = createMap(), slam = createSlam(map, start, { maxScans: 5 });
  for (let i = 0; i < 10; i++) observeSlam(slam, map, sense(start, walls), start, SENSOR, i * .1);
  assert.equal(slam.records.length, 5); assert.equal(slam.limited, true);
  assert.equal(slamSummary(slam).status, 'limite_de_memoria');
});

test('el cierre de bucle preserva seguridad y sensores sin forzar cobertura', () => {
  const { report } = evaluateScenario(SCENARIOS.partition, 24000, { odometry: {}, seed: 7, loopClosure: true });
  assert.ok(report.slam.scans > 0 && report.slam.submaps > 0);
  assert.equal(report.estado, 'completado', JSON.stringify(report));
  assert.equal(report.fronteras_pendientes, 0);
  assert.equal(report.colisiones, 0);
  assert.equal(report.pasos_con_huella_fuera_del_espacio_libre_conocido, 0);
  assert.equal(report.odometria, 'ruidosa');
  assert.ok(report.error_odom_m < .5, JSON.stringify(report));
});
