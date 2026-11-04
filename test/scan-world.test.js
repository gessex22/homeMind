import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { buildScanWorld, senseScanWorld, advanceScanWorld, huellaTransitable } from '../src/scan-world.js';
import { extractGlbGeometry } from '../src/glb-geometry.js';
import { createScanSession } from '../src/scan-session.js';
import { createMap, cell, integrate, FREE, UNKNOWN, OCCUPIED } from '../src/mapping.js';
import { footprintClear } from '../src/planner.js';
import { tileCalibration } from '../src/scan-calibration.js';
import { createSlam, observeSlam, exportSlam } from '../src/slam.js';
const floor = (x1, z1, x2, z2) => [x1, 0, z1, x2, 0, z1, x2, 0, z2, x1, 0, z1, x2, 0, z2, x1, 0, z2];
function synthetic() {
  return { triangles: new Float32Array([...floor(-2, -2, 0, 2), ...floor(.4, -2, 2, 2),
    1, 0, -2, 1, 1, -2, 1, 1, 2, 1, 0, -2, 1, 1, 2, 1, 0, 2]), bounds: { minX: -2, maxX: 2, minZ: -2, maxZ: 2, maxY: 1 } };
}
test('dirección manual avanza, se detiene al soltar, retrocede y gira conservando el mapa', () => {
  const geometry = { triangles: new Float32Array(floor(-2, -2, 2, 2)), bounds: { minX: -2, maxX: 2, minZ: -2, maxZ: 2, maxY: 1 } };
  const s = createScanSession(geometry, { noisy: false }), start = s.pose;
  for (let i = 0; i < 12; i++) s.step({ forward: true });
  assert.ok(Math.abs(s.pose.z - start.z + .2) < 1e-6);
  const stopped = s.pose, known = s.nav.map.filter(v => v !== UNKNOWN).length;
  for (let i = 0; i < 12; i++) s.step({});
  assert.deepEqual(s.pose, stopped);
  for (let i = 0; i < 12; i++) s.step({ back: true });
  assert.ok(Math.hypot(s.pose.x - start.x, s.pose.z - start.z) < 1e-6);
  for (let i = 0; i < 60; i++) s.step({ left: true });
  assert.ok(Math.abs(s.pose.yaw - Math.PI / 2) < 1e-6);
  for (let i = 0; i < 60; i++) s.step({ right: true });
  assert.ok(Math.abs(s.pose.yaw) < 1e-6);
  assert.ok(s.nav.map.filter(v => v !== UNKNOWN).length >= known);
  assert.equal(s.nav.state, 'pausado');
  s.nav.start(); s.step(); assert.notEqual(s.nav.state, 'pausado');
});
test('la dirección manual respeta paredes y permite retroceder sin teletransportar al robot', () => {
  const geometry = { triangles: new Float32Array([...floor(-2, -2, 2, 2),
    -2, 0, -1.6, 2, 0, -1.6, 2, 1, -1.6, -2, 0, -1.6, 2, 1, -1.6, -2, 1, -1.6]), bounds: { minX: -2, maxX: 2, minZ: -2, maxZ: 2, maxY: 1 } };
  const s = createScanSession(geometry, { noisy: false });
  for (let i = 0; i < 240 && !s.diagnostics.contacts; i++) {
    const result = s.step({ forward: true }); assert.equal(result.groundStop, false); assert.ok(huellaTransitable(s.world, s.pose));
  }
  assert.ok(s.diagnostics.contacts > 0);
  const contactPose = s.pose;
  for (let i = 0; i < 60; i++) s.step({ forward: true });
  assert.ok(Math.hypot(s.pose.x - contactPose.x, s.pose.z - contactPose.z) < 1e-5);
  for (let i = 0; i < 24; i++) s.step({ back: true });
  assert.ok(s.pose.z > contactPose.z + .3);
});
test('la baldosa ajusta distancias, excluyendo la junta, y rechaza puntos inválidos', () => {
  assert.ok(Math.abs(tileCalibration({ x: 0, y: 0, z: 0 }, { x: .3, y: 0, z: 0 }).scale - 1.09) < 1e-9);
  assert.throws(() => tileCalibration({ x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0 }));
});
test('conserva el hueco del suelo y detiene la huella sin inventar contacto', () => {
  const w = buildScanWorld(synthetic()), pose = { x: -.5, z: 0, yaw: -Math.PI / 2 };
  assert.equal(w.map[cell(.2, 0, w.map)], UNKNOWN);
  const moved = advanceScanWorld(w, pose, { forward: true }, 1);
  assert.equal(moved.groundStop, true); assert.equal(moved.collision, false);
  assert.ok(huellaTransitable(w, moved, w.radius)); assert.ok(moved.x < .2);
});
test('la profundidad ve la pared tras el hueco sin despejar el suelo ausente; SLAM conserva ese límite', () => {
  const w = buildScanWorld(synthetic()), pose = { x: -.5, z: 0, yaw: -Math.PI / 2 };
  const rays = senseScanWorld(w, pose, { rays: 1, fov: 0, range: 3, ground: true });
  assert.equal(rays[0].kind, 'wall'); assert.ok(Math.abs(rays[0].distance - 1.5) < .001); assert.ok(rays[0].freeDistance > .5 && rays[0].freeDistance < 1.5);
  const map = createMap(w.grid); integrate(map, pose, rays); integrate(map, pose, rays);
  assert.equal(map[cell(.2, 0, map)], UNKNOWN); assert.equal(map[cell(1.005, 0, map)], OCCUPIED);
  const slam = createSlam(map, pose); observeSlam(slam, map, rays, pose, { fov: 0 }, 0, false);
  const saved = exportSlam(slam); assert.ok(saved.observations[0].freeDistances[0] < 1.5);
});
test('un hueco chico de captura se puentea; el de 40 cm sigue deteniendo', () => {
  const geometry = { triangles: new Float32Array([...floor(-2, -2, 0, 2), ...floor(.2, -2, 2, 2)]), bounds: { minX: -2, maxX: 2, minZ: -2, maxZ: 2, maxY: 1 } };
  const w = buildScanWorld(geometry);
  assert.ok(w.stats.puentes > 0);
  const moved = advanceScanWorld(w, { x: -.5, z: 0, yaw: -Math.PI / 2 }, { forward: true }, 1);
  assert.equal(moved.groundStop, false);
  assert.equal(moved.collision, false);
  assert.ok(moved.x > .3, JSON.stringify(moved));
});
test('colisión física con obstáculo se distingue del límite de captura', () => {
  const w = buildScanWorld(synthetic()), pose = { x: 1.5, z: 0, yaw: Math.PI / 2 };
  const moved = advanceScanWorld(w, pose, { forward: true }, 1);
  assert.equal(moved.collision, true); assert.equal(moved.groundStop, false); assert.ok(footprintClear(w.map, moved, w.radius));
});
test('rechaza navegar si el suelo no alcanza para la huella del robot', () => {
  const geometry = { triangles: new Float32Array(floor(-.08, -1, .08, 1)), bounds: { minX: -.08, maxX: .08, minZ: -1, maxZ: 1, maxY: .1 } };
  assert.equal(buildScanWorld(geometry).initialPose, null);
  assert.throws(() => createScanSession(geometry), /huella de 20 cm/);
});
test('el GLB real permite simular sensores sin precargar el mapa y cada movimiento respeta la huella', () => {
  const bytes = readFileSync(new URL('../public/scans/polycam-2026-10-28/10_28_2026.glb', import.meta.url));
  const geometry = extractGlbGeometry(bytes); assert.equal(geometry.triangles.length / 9, 118302);
  const s = createScanSession(geometry, { noisy: true });
  assert.ok(s.world.stats.freeAreaM2 > 4); assert.ok(s.nav.map.filter(v => v === FREE).length < 100);
  s.nav.start();
  for (let i = 0; i < 3600; i++) {
    const r = s.step(); assert.ok(huellaTransitable(s.world, s.pose, s.world.radius)); if (r.stopped) break;
  }
  assert.ok(s.diagnostics.steps > 60); assert.ok(s.nav.map.filter(v => v !== UNKNOWN).length > 100);
  assert.throws(() => extractGlbGeometry(new Uint8Array(20)), /inválido/);
});
