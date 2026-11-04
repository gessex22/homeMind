import test from 'node:test';
import assert from 'node:assert/strict';
import { createNavigator } from '../src/navigation.js';
import { sense, SENSOR } from '../src/sensor.js';
import { SCENARIOS, solidos } from '../src/world.js';
import { advance, STEP } from '../src/physics.js';
import { createOdometry, readMotion, SENSORES_RUIDOSOS } from '../src/odometry.js';
import { mulberry32 } from '../src/noise.js';

function drive(nav, scenario, { noisy = true, seed = 7, maxSteps = 24000 } = {}) {
  const adapter = createOdometry({ x: 0, z: 0, yaw: 0 }, noisy ? { ...SENSORES_RUIDOSOS, seed } : { seed });
  const sensor = noisy ? { ...SENSOR, noise: .02, random: mulberry32((seed ^ 0x51f3) >>> 0) } : SENSOR;
  let pose = { x: 0, z: 0, yaw: 0, collision: false }, motion = null, collisions = 0;
  nav.start();
  for (let step = 0; step < maxSteps && !nav.terminal; step++) {
    const rays = step % 6 === 0 ? sense(pose, scenario.walls, sensor) : null;
    const command = nav.step(STEP, { rays, motion });
    const prev = { x: pose.x, z: pose.z, yaw: pose.yaw };
    const next = advance(pose, command, STEP, solidos(scenario));
    motion = readMotion(adapter, next, prev, command, STEP, next.collision);
    collisions += Number(Boolean(next.collision));
    pose = next;
  }
  return { pose, collisions };
}

test('destino inválido se rechaza y limpiar vuelve a exploración', () => {
  const nav = createNavigator(SCENARIOS.partition.mapConfig, { noisy: false });
  assert.throws(() => nav.setDestination({ x: NaN, z: 0 }), RangeError);
  nav.setDestination({ x: 1, z: -1 });
  assert.deepEqual(nav.destination, { x: 1, z: -1 });
  nav.setDestination(null);
  assert.equal(nav.destination, null);
});

test('va a un destino conocido al otro lado de la abertura (exacto)', { timeout: 120000 }, () => {
  const nav = createNavigator(SCENARIOS.partition.mapConfig, { noisy: false, seed: 7 });
  nav.setDestination({ x: 0, z: -2 });
  const { collisions } = drive(nav, SCENARIOS.partition, { noisy: false });
  assert.equal(nav.state, 'destino_alcanzado');
  assert.equal(collisions, 0);
  assert.ok(Math.hypot(nav.belief.x - 0, nav.belief.z + 2) < .4);
});

test('sin ruta conocida explora primero y después llega (ruidoso)', { timeout: 180000 }, () => {
  const nav = createNavigator(SCENARIOS.partition.mapConfig, { seed: 7 });
  nav.setDestination({ x: 0, z: -2 });
  const { collisions } = drive(nav, SCENARIOS.partition, {});
  assert.equal(nav.state, 'destino_alcanzado');
  assert.equal(collisions, 0);
  assert.ok(Math.hypot(nav.belief.x - 0, nav.belief.z + 2) < .45);
  assert.ok(nav.summary().correcciones_scan > 0);
});

test('destino tras la abertura de 0,3 m nunca se declara alcanzado', { timeout: 120000 }, () => {
  const nav = createNavigator(SCENARIOS.narrow.mapConfig, { noisy: false, seed: 7 });
  nav.setDestination({ x: 0, z: -2 });
  const { collisions } = drive(nav, SCENARIOS.narrow, { noisy: false });
  assert.notEqual(nav.state, 'destino_alcanzado');
  assert.equal(nav.state, 'limitado');
  assert.equal(collisions, 0);
});

test('puede recibir un destino después de completar la exploración', { timeout: 120000 }, () => {
  const nav = createNavigator(SCENARIOS.partition.mapConfig, { noisy: false, seed: 7 });
  const first = drive(nav, SCENARIOS.partition, { noisy: false });
  assert.equal(nav.state, 'completado');
  nav.setDestination({ x: 0, z: 0 });
  const second = drive(nav, SCENARIOS.partition, { noisy: false });
  assert.equal(nav.state, 'destino_alcanzado');
  assert.ok(Math.hypot(nav.belief.x, nav.belief.z) < .4);
  assert.equal(first.collisions + second.collisions, 0);
});
