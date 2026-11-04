import test from 'node:test';
import assert from 'node:assert/strict';
import { lidarViewRays } from '../src/lidar-view.js';
import { observeSweep, sweepCovered } from '../src/sweep-coverage.js';
import { createExplorer, startExplorer, pauseExplorer, setDestination } from '../src/explorer.js';
import { createNavigator } from '../src/navigation.js';
import { sense, SENSOR } from '../src/sensor.js';
import { SCENARIOS, solidos } from '../src/world.js';
import { advance, STEP } from '../src/physics.js';
import { createOdometry, readMotion } from '../src/odometry.js';

test('LiDAR view rotates once with belief and ignores physical ray headings', () => {
  const scan = { belief: { x: 2, z: 3, yaw: Math.PI / 2 },
    rays: [{ kind: 'wall', distance: 2, angle: -999 }] };
  const [ray] = lidarViewRays(scan);
  assert.ok(Math.abs(ray.x) < 1e-9);
  assert.ok(Math.abs(ray.z - 3) < 1e-9);
  scan.rays[0].angle = 999;
  assert.deepEqual(lidarViewRays(scan), [ray]);
  assert.deepEqual(lidarViewRays({ ...scan, rays: [{ kind: 'invalid', distance: null }] }), []);
});

test('coverage requires real received headings and resets after translation', () => {
  let c = observeSweep(null, { x: 0, z: 0, yaw: 0 }, SENSOR.fov);
  for (let i = 0; i < 50; i++) c = observeSweep(c, { x: 0, z: 0, yaw: 0 }, SENSOR.fov);
  assert.equal(sweepCovered(c), false);
  for (let deg = 9; deg <= 297; deg += 9) c = observeSweep(c, { x: 0, z: 0, yaw: deg * Math.PI / 180 }, SENSOR.fov);
  assert.equal(sweepCovered(c), true);
  assert.equal(sweepCovered(observeSweep(c, { x: 1, z: 0, yaw: 0 }, SENSOR.fov)), false);
});

test('pausing twice preserves the partial scan and the active destination route', () => {
  const e = createExplorer(); startExplorer(e); e.scanAngle = 1.2;
  pauseExplorer(e); pauseExplorer(e); startExplorer(e);
  assert.equal(e.state, 'barrido'); assert.equal(e.scanAngle, 1.2);
  e.state = 'destino'; e.destino = { x: 1, z: 2 }; e.route = [1, 2, 3];
  pauseExplorer(e); startExplorer(e);
  assert.equal(e.state, 'destino'); assert.deepEqual(e.route, [1, 2, 3]);
  pauseExplorer(e); setDestination(e, { x: -1, z: -2 }); startExplorer(e);
  assert.equal(e.state, 'planificando'); assert.deepEqual(e.destino, { x: -1, z: -2 });
  e.state = 'destino'; pauseExplorer(e); setDestination(e, null); startExplorer(e);
  assert.equal(e.state, 'planificando'); assert.equal(e.destino, null);
});

function firstSweep(adaptiveSweeps, withScans = true) {
  const scenario = SCENARIOS.empty;
  const nav = createNavigator(scenario.mapConfig, { noisy: false, adaptiveSweeps });
  const reader = createOdometry({ x: 0, z: 0, yaw: 0 }, { seed: 7 });
  let pose = { x: 0, z: 0, yaw: 0 }, motion = null, turns = 0;
  nav.start();
  for (let i = 0; i < 1000 && nav.state === 'barrido'; i++) {
    const rays = withScans && i % 6 === 0 ? sense(pose, scenario.walls) : null;
    const command = nav.step(STEP, { motion, rays });
    const next = advance(pose, command, STEP, solidos(scenario));
    motion = readMotion(reader, next, pose, command, STEP, next.collision);
    if (command.left || command.right) turns++;
    assert.equal(Boolean(next.collision), false); pose = next;
  }
  return turns;
}

test('80-degree scans cover the circle with less body rotation; absent scans do not shortcut it', () => {
  const full = firstSweep(false), adaptive = firstSweep(true);
  assert.ok(adaptive < full - 20, { full, adaptive });
  assert.ok(adaptive * 1.5 >= 280, `rotation=${adaptive * 1.5} degrees`);
  assert.equal(firstSweep(true, false), firstSweep(false, false));
});

test('adaptive frontal scanning still explores the room without contacts or hidden geometry', () => {
  const scenario = SCENARIOS.empty;
  const nav = createNavigator(scenario.mapConfig, { noisy: false, adaptiveSweeps: true });
  const reader = createOdometry({ x: 0, z: 0, yaw: 0 }, { seed: 7 });
  let pose = { x: 0, z: 0, yaw: 0 }, motion = null;
  nav.start();
  for (let i = 0; i < 24000 && !nav.terminal; i++) {
    const rays = i % 6 === 0 ? sense(pose, scenario.walls) : null;
    const command = nav.step(STEP, { rays, motion });
    const next = advance(pose, command, STEP, solidos(scenario));
    assert.equal(Boolean(next.collision), false);
    motion = readMotion(reader, next, pose, command, STEP, next.collision); pose = next;
  }
  assert.equal(nav.state, 'completado');
  assert.equal(nav.fronts().length, 0);
});
