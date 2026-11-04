import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { destinationFromPointer, selectMapDestination } from '../src/scan-destination.js';
import { createNavigator } from '../src/navigation.js';
import { cell, FREE, OCCUPIED } from '../src/mapping.js';
import { createScanSession } from '../src/scan-session.js';
import { extractGlbGeometry } from '../src/glb-geometry.js';
const grid = { originX: -2, originZ: -4, resolution: .1, width: 40, height: 80 };
function knownNav() {
  const nav = createNavigator(grid, { noisy: false, initialPose: { x: -1, z: 0, yaw: 0 }, footprint: .1 });
  nav.map.fill(FREE); nav.map.logodds.fill(-4); nav.map.revision++;
  return nav;
}
test('el clic se transforma correctamente incluso con bandas vacías en el canvas', () => {
  const rect = { left: 10, top: 20, width: 800, height: 400 };
  assert.deepEqual(destinationFromPointer(410, 220, rect, grid), { x: 0, z: 0 });
  assert.equal(destinationFromPointer(20, 220, rect, grid), null);
  assert.equal(destinationFromPointer(250, 220, rect, grid), null);
  assert.equal(destinationFromPointer(410, 900, rect, grid), null);
});
test('seleccionar y reemplazar destinos cambia la ruta sin mover ni reiniciar el mapa', () => {
  const nav = knownNav(), before = nav.belief, map = nav.map;
  const first = selectMapDestination(nav, { x: 1, z: -1 }, .1);
  assert.ok(first.route.length > 0); assert.equal(nav.state, 'planificando');
  nav.step(1 / 60); assert.equal(nav.state, 'destino');
  const second = selectMapDestination(nav, { x: -1, z: 2 }, .1);
  assert.deepEqual(nav.destination, second.goal); assert.notDeepEqual(first.goal, second.goal);
  assert.equal(nav.state, 'planificando'); assert.deepEqual(nav.explorer.route, []);
  assert.deepEqual(nav.belief, before); assert.equal(nav.map, map);
  nav.step(1 / 60); nav.setDestination(null);
  assert.equal(nav.destination, null); assert.doesNotThrow(() => nav.step(1 / 60));
});
test('rechaza desconocidos, obstáculos y destinos sin conexión usando solo el mapa del robot', () => {
  const unknown = createNavigator(grid, { noisy: false });
  assert.throws(() => selectMapDestination(unknown, { x: 1, z: -1 }, .1), /suelo blanco/);
  const nav = knownNav(); nav.map[cell(1, -1, nav.map)] = OCCUPIED;
  assert.throws(() => selectMapDestination(nav, { x: 1, z: -1 }, .1), /suelo blanco/);
  assert.throws(() => selectMapDestination(nav, null, .1), /dentro del mapa/);
  nav.map.fill(FREE);
  for (let row = 0; row < grid.height; row++) nav.map[row * grid.width + 20] = OCCUPIED;
  nav.map.revision++;
  assert.throws(() => selectMapDestination(nav, { x: 1, z: 1 }, .1), /ruta conocida/);
  assert.equal(nav.destination, null);
});
test('el escaneo de varias habitaciones permite regresar a un punto elegido del mapa', () => {
  const geometry = extractGlbGeometry(readFileSync(new URL('../public/scans/polycam-2026-10-28/10_28_2026-varias-habitaciones.glb', import.meta.url)));
  const s = createScanSession(geometry, { noisy: false });
  const base = { ...s.nav.belief }; s.nav.start();
  for (let i = 0; i < 1800; i++) if (s.step().stopped) break;
  assert.ok(s.diagnostics.distance > .5);
  const selected = selectMapDestination(s.nav, base, s.world.radius);
  for (let i = 0; i < 7200; i++) if (s.step().stopped) break;
  assert.equal(s.nav.state, 'destino_alcanzado');
  assert.ok(Math.hypot(s.pose.x - selected.goal.x, s.pose.z - selected.goal.z) < .4);
  assert.equal(s.diagnostics.contacts, 0);
});

test('ruta del cuarto superior al punto inferior de la imagen con ruido y frente a la pared', () => {
  const geometry = extractGlbGeometry(readFileSync(new URL('../public/scans/polycam-2026-10-28/10_28_2026-varias-habitaciones.glb', import.meta.url)));
  const s = createScanSession(geometry, { noisy: true, seed: 7 });
  s.nav.start();
  for (let i = 0; i < 1800; i++) if (s.step().stopped) break;
  // Approximate positions from the user's map image; the map itself must
  // have been observed normally before either click is accepted.
  selectMapDestination(s.nav, { x: -.5643, z: -5.8695 }, s.world.radius);
  for (let i = 0; i < 7200; i++) if (s.step().stopped) break;
  assert.equal(s.nav.state, 'destino_alcanzado');
  for (let i = 0; i < 240; i++) {
    const delta = Math.atan2(Math.sin(-1.3 - s.nav.belief.yaw), Math.cos(-1.3 - s.nav.belief.yaw));
    if (Math.abs(delta) < .02) break;
    s.step(delta > 0 ? { left: true } : { right: true });
  }
  const selected = selectMapDestination(s.nav, { x: -1.188, z: -2.3188 }, s.world.radius);
  for (let i = 0; i < 7200; i++) if (s.step().stopped) break;
  assert.equal(s.nav.state, 'destino_alcanzado');
  assert.ok(Math.hypot(s.pose.x - selected.goal.x, s.pose.z - selected.goal.z) < .15);
  assert.equal(s.diagnostics.contacts, 0);
});

test('regresa del cuarto superior tras una exploración larga sin encerrarse con celdas evitadas', () => {
  const geometry = extractGlbGeometry(readFileSync(new URL('../public/scans/polycam-2026-10-28/10_28_2026-varias-habitaciones.glb', import.meta.url)));
  const s = createScanSession(geometry, { noisy: true, seed: 7 });
  s.nav.start();
  for (let i = 0; i < 7200; i++) if (s.step().stopped) break;
  selectMapDestination(s.nav, { x: -.5643, z: -5.8695 }, s.world.radius);
  for (let i = 0; i < 2400; i++) if (s.step().stopped) break;
  assert.equal(s.nav.state, 'destino_alcanzado');
  const selected = selectMapDestination(s.nav, { x: -1.188, z: -2.3188 }, s.world.radius);
  for (let i = 0; i < 2400; i++) if (s.step().stopped) break;
  assert.equal(s.nav.state, 'destino_alcanzado');
  assert.ok(s.nav.explorer.avoid.size > 0, 'el recorrido ejercita la recuperación y su memoria de celdas evitadas');
  assert.ok(Math.hypot(s.pose.x - selected.goal.x, s.pose.z - selected.goal.z) < .15);
  assert.equal(s.diagnostics.groundStops, 0);
});
