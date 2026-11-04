import test from 'node:test';
import assert from 'node:assert/strict';
import { createNavigator } from '../src/navigation.js';
import { sense, SENSOR } from '../src/sensor.js';
import { detectObjects } from '../src/camera.js';
import { SCENARIOS, solidos } from '../src/world.js';
import { advance, STEP } from '../src/physics.js';
import { createOdometry, readMotion, SENSORES_RUIDOSOS } from '../src/odometry.js';
import { mulberry32 } from '../src/noise.js';
import { createMap, integrate, gridOf } from '../src/mapping.js';

const scene = SCENARIOS.casamoderna;

function drive(nav, { noisy = true, seed = 7, maxSteps = 24000, start = { x: 0, z: 0, yaw: 0 }, marcas = false } = {}) {
  const adapter = createOdometry(start, noisy ? { ...SENSORES_RUIDOSOS, seed } : { seed });
  const sensor = noisy ? { ...SENSOR, noise: .02, random: mulberry32((seed ^ 0x51f3) >>> 0) } : SENSOR;
  let pose = { ...start, collision: false }, motion = null, collisions = 0;
  nav.start();
  for (let step = 0; step < maxSteps && !nav.terminal; step++) {
    const rays = step % 6 === 0 ? sense(pose, scene.walls, sensor) : null;
    const detecciones = rays && marcas ? detectObjects(pose, [...scene.boards, ...scene.muebles], scene.walls) : null;
    const command = nav.step(STEP, { rays, motion, detecciones });
    const prev = { x: pose.x, z: pose.z, yaw: pose.yaw };
    const next = advance(pose, command, STEP, solidos(scene));
    motion = readMotion(adapter, next, prev, command, STEP, next.collision);
    collisions += Number(Boolean(next.collision));
    pose = next;
  }
  return { pose, collisions };
}

test('la cámara simulada ve objetos frontales y no a través de paredes', () => {
  const frente = detectObjects({ x: -3.2, z: 1.8, yaw: Math.PI }, scene.boards, scene.walls);
  assert.ok(frente.some(d => d.label === 'rojo'));
  const espalda = detectObjects({ x: -3.2, z: 1.8, yaw: 0 }, scene.boards, scene.walls);
  assert.ok(!espalda.some(d => d.label === 'rojo' && d.dist < 2.5));
  const lejos = detectObjects({ x: -3.2, z: -3, yaw: Math.PI }, scene.boards, scene.walls);
  assert.ok(!lejos.some(d => d.label === 'rojo'));
});

function mapaPrevioCompleto() {
  // Prior map fixture: scans integrated from scripted coverage poses, the
  // map a thorough earlier mission would have saved. Isolates the visual
  // tie-break from explorer coverage limits in big rooms.
  const map = createMap(scene.mapConfig);
  const poses = [
    { x: -3.2, z: 1.5 }, { x: 0, z: 1.5 }, { x: 3.2, z: 1.5 },
    { x: -2.5, z: -2 }, { x: 2.5, z: -2 }, { x: 0, z: -2 },
    { x: -3.2, z: .5 }, { x: 3.2, z: .5 }, { x: -4, z: -3 }, { x: 4, z: -3 },
  ];
  for (const p of poses) for (const yaw of [0, Math.PI / 2, Math.PI, -Math.PI / 2])
    integrate(map, { ...p, yaw }, sense({ ...p, yaw }, scene.walls, SENSOR));
  const g = gridOf(map);
  return { formato: 'homemind-mapa', version: 1, grid: { originX: g.originX, originZ: g.originZ, resolution: g.resolution, width: g.width, height: g.height }, celdas: Array.from(map) };
}

test('las marcas visuales desempatan el cuarto gemelo al relocalizar', { timeout: 180000 }, () => {
  const saved = mapaPrevioCompleto();
  const priorMarcas = [...scene.boards, ...scene.muebles].map(b => ({ label: b.label, x: b.x, z: b.z }));
  // Twin bedroom B (azul). Geometry also fits bedroom A (rojo) perfectly.
  const real = { x: 3.2, z: 1.8, yaw: .6 };
  const nav = createNavigator(scene.mapConfig, { noisy: false, priorMap: saved, priorMarcas });
  const { pose, collisions } = drive(nav, { noisy: false, start: real, marcas: true });
  assert.equal(nav.summary().relocalizacion.found, true, JSON.stringify(nav.summary().relocalizacion));
  assert.equal(nav.summary().marcas_previas, 15);
  assert.equal(collisions, 0);
  assert.ok(Math.hypot(nav.belief.x - pose.x, nav.belief.z - pose.z) < .25, JSON.stringify(nav.summary()));
});
