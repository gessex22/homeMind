import test from 'node:test';
import assert from 'node:assert/strict';
import { createNavigator, NAV_TERMINAL } from '../src/navigation.js';
import { sense, SENSOR } from '../src/sensor.js';
import { SCENARIOS, solidos } from '../src/world.js';
import { advance, STEP } from '../src/physics.js';
import { createOdometry, readMotion, SENSORES_RUIDOSOS } from '../src/odometry.js';
import { mulberry32 } from '../src/noise.js';
import { counts } from '../src/mapping.js';

// Host-side adapter: physics + sensors produce rays and encoder measurements;
// truth never enters the engine.
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

test('API: inactivo no se mueve, pausa congela el comando y reset limpia', () => {
  const nav = createNavigator(SCENARIOS.partition.mapConfig, { noisy: false });
  assert.equal(nav.state, 'inactivo');
  assert.equal(nav.terminal, false);
  assert.deepEqual(nav.step(STEP, {}), { forward: false, back: false, left: false, right: false });
  nav.start();
  assert.equal(nav.state, 'barrido');
  nav.pause();
  assert.equal(nav.state, 'pausado');
  assert.deepEqual(nav.step(STEP, { motion: { avance: 0, giro: 0, contact: false } }), { forward: false, back: false, left: false, right: false });
  const summary = nav.summary();
  assert.equal(summary.estado, 'pausado');
  assert.ok(summary.celdas.desconocido > 0 && summary.tiempo_s >= 0);
  assert.throws(() => nav.step(0, {}), RangeError);
  nav.reset();
  assert.equal(nav.state, 'inactivo');
  const cells = counts(nav.map);
  assert.ok(cells.desconocido > nav.map.length * .99);
  assert.ok(NAV_TERMINAL.includes('completado'));
});

test('soltarlo en una partición exacta la explora y mapea sin ruta previa', { timeout: 120000 }, () => {
  const nav = createNavigator(SCENARIOS.partition.mapConfig, { noisy: false, seed: 7 });
  const { pose, collisions } = drive(nav, SCENARIOS.partition, { noisy: false });
  assert.equal(nav.state, 'completado');
  assert.equal(nav.fronts().length, 0);
  assert.equal(collisions, 0);
  const belief = nav.belief;
  assert.ok(Math.hypot(belief.x - pose.x, belief.z - pose.z) < .1);
  assert.ok(nav.summary().conocidas_pct > 45);
});

test('soltarlo con sensores ruidosos acota la deriva y completa', { timeout: 120000 }, () => {
  const nav = createNavigator(SCENARIOS.partition.mapConfig, { seed: 7 });
  const { pose, collisions } = drive(nav, SCENARIOS.partition, {});
  assert.equal(nav.state, 'completado');
  assert.equal(collisions, 0);
  const summary = nav.summary();
  assert.ok(summary.correcciones_scan > 0);
  assert.ok(Math.hypot(summary.creencia.x - pose.x, summary.creencia.z - pose.z) < .25, JSON.stringify(summary));
});

test('soltarlo en la casa ruidosa la recorre entera sin chocar', { timeout: 180000 }, () => {
  const nav = createNavigator(SCENARIOS.casa.mapConfig, { seed: 7 });
  const { pose, collisions } = drive(nav, SCENARIOS.casa, {});
  assert.equal(nav.state, 'completado');
  assert.equal(collisions, 0);
  assert.ok(Math.hypot(nav.belief.x - pose.x, nav.belief.z - pose.z) < .35);
});
