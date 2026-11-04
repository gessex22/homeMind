import test from 'node:test';
import assert from 'node:assert/strict';
import { alignScan, blendPose } from '../src/scanmatch.js';
import { sense } from '../src/sensor.js';
import { createMap, integrate } from '../src/mapping.js';
import { SCENARIOS } from '../src/world.js';
import { evaluateScenario } from '../scripts/evaluate-room.mjs';

const truth = { x: 0, z: 0, yaw: 0 };

function mappedScan() {
  const map = createMap();
  const rays = sense(truth, SCENARIOS.partition.walls);
  integrate(map, truth, rays);
  integrate(map, truth, rays);
  return { map, rays };
}

test('recupera una pose con deriva frente a una puerta', () => {
  const at = { x: 0, z: .5, yaw: Math.PI / 2 };
  const map = createMap(SCENARIOS.casa.mapConfig);
  const rays = sense(at, SCENARIOS.casa.walls);
  integrate(map, at, rays);
  integrate(map, at, rays);
  const fix = alignScan(map, rays, { x: .15, z: .4, yaw: Math.PI / 2 + .04 });
  assert.equal(fix.corregido, true);
  assert.ok(Math.abs(fix.pose.x) <= .075 && Math.abs(fix.pose.z - .5) <= .05);
  assert.ok(Math.abs(fix.pose.yaw - Math.PI / 2) <= .021);
});

test('sin impactos de pared no corrige', () => {
  const map = createMap();
  const rays = sense({ x: 0, z: 3, yaw: Math.PI }, SCENARIOS.large.walls);
  assert.ok(rays.every(r => r.kind === 'floor'));
  const fix = alignScan(map, rays, { x: .1, z: 3, yaw: Math.PI });
  assert.equal(fix.corregido, false);
  assert.deepEqual(fix.pose, { x: .1, z: 3, yaw: Math.PI });
});

test('sobre un mapa vacío no corrige', () => {
  const rays = sense(truth, SCENARIOS.partition.walls);
  const fix = alignScan(createMap(), rays, { x: .1, z: 0, yaw: 0 });
  assert.equal(fix.corregido, false);
  assert.deepEqual(fix.pose, { x: .1, z: 0, yaw: 0 });
});

test('una pose ya alineada se deja quieta', () => {
  const { map, rays } = mappedScan();
  const fix = alignScan(map, rays, { ...truth });
  assert.equal(fix.corregido, false);
  assert.deepEqual(fix.pose, truth);
});

test('la mezcla aplica una fracción y envuelve el yaw', () => {
  assert.deepEqual(blendPose({ x: 0, z: 0, yaw: 0 }, { x: 1, z: 0, yaw: 0 }), { x: .3, z: 0, yaw: 0 });
  const wrapped = blendPose({ x: 0, z: 0, yaw: Math.PI - .1 }, { x: 0, z: 0, yaw: -Math.PI + .1 });
  assert.ok(Math.abs(wrapped.yaw - (Math.PI - .04)) < 1e-9);
});

test('correcciones locales completan una partición con ruido sin pasos inseguros', { timeout: 120000 }, () => {
  const { report } = evaluateScenario(SCENARIOS.partition, 24000, { odometry: {}, seed: 7 });
  assert.equal(report.estado, 'completado', JSON.stringify(report));
  assert.equal(report.fronteras_pendientes, 0);
  assert.equal(report.colisiones, 0);
  assert.equal(report.pasos_con_huella_fuera_del_espacio_libre_conocido, 0);
  assert.ok(report.correcciones_scan > 0);
  assert.ok(report.error_odom_m < .25, JSON.stringify(report));
  assert.ok(report.error_yaw_grados < 5, JSON.stringify(report));
});

test('scan matching local no garantiza recuperar una sala grande con deriva', { timeout: 180000 }, () => {
  const { report } = evaluateScenario(SCENARIOS.large, 36000, { odometry: {}, seed: 7 });
  assert.equal(report.estado, 'limitado', JSON.stringify(report));
  assert.equal(report.aprobado, false);
  assert.ok(report.fronteras_pendientes > 0);
  assert.equal(report.colisiones, 0);
  assert.equal(report.pasos_con_huella_fuera_del_espacio_libre_conocido, 0);
  assert.ok(report.correcciones_scan > 0);
});
