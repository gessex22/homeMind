import test from 'node:test';
import assert from 'node:assert/strict';
import { createNavigator } from '../src/navigation.js';
import { sense, SENSOR } from '../src/sensor.js';
import { SCENARIOS, solidos } from '../src/world.js';
import { advance, STEP } from '../src/physics.js';
import { createOdometry, readMotion, SENSORES_RUIDOSOS } from '../src/odometry.js';
import { mulberry32 } from '../src/noise.js';
import { frontiers } from '../src/planner.js';

test('cerrar las fronteras residuales verificadas convierte limitado en completado', { timeout: 180000 }, () => {
  const scene = SCENARIOS.casamoderna;
  const start = scene.inicio, seed = 42;
  const nav = createNavigator(scene.mapConfig, { noisy: true, seed, initialPose: start });
  const adapter = createOdometry(start, { ...SENSORES_RUIDOSOS, seed });
  const sensor = { ...SENSOR, noise: .02, random: mulberry32((seed ^ 0x51f3) >>> 0) };
  let pose = { ...start, collision: false }, motion = null;
  nav.start();
  for (let step = 0; step < 24000 && !nav.terminal; step++) {
    const rays = step % 6 === 0 ? sense(pose, scene.walls, sensor) : null;
    const command = nav.step(STEP, { rays, motion });
    const prev = { x: pose.x, z: pose.z, yaw: pose.yaw };
    const next = advance(pose, command, STEP, solidos(scene));
    motion = readMotion(adapter, next, prev, command, STEP, next.collision);
    pose = next;
  }
  assert.equal(nav.state, 'limitado', JSON.stringify(nav.summary()));
  const pendientes = frontiers(nav.map);
  assert.ok(pendientes.length > 0);
  // Partial closure must not complete: only all of them does.
  nav.closeFrontiers(pendientes.slice(0, 1));
  assert.notEqual(nav.state, 'completado');
  nav.closeFrontiers(pendientes);
  assert.equal(nav.state, 'completado', JSON.stringify(nav.summary()));
  assert.ok(nav.summary().zonas_cerradas >= pendientes.length);
});
