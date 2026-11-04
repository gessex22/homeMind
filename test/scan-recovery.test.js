import test from 'node:test';
import assert from 'node:assert/strict';
import { createScanSession } from '../src/scan-session.js';
import { FREE, OCCUPIED } from '../src/mapping.js';
import { huellaTransitable } from '../src/scan-world.js';
import { selectMapDestination } from '../src/scan-destination.js';

function room() {
  return createScanSession({
    triangles: new Float32Array([-2, 0, -2, 2, 0, -2, 2, 0, 2, -2, 0, -2, 2, 0, 2, -2, 0, 2]),
    bounds: { minX: -2, maxX: 2, minZ: -2, maxZ: 2, maxY: 1 },
  }, { noisy: false });
}

test('el interbloqueo de suelo llega al motor y permite retirarse sin pausar ni reiniciar', () => {
  const s = room(), map = s.nav.map;
  // A mistaken free-space estimate exercises the physical floor interlock.
  // The controller still receives only its map and the measured stop event.
  map.fill(FREE); map.logodds.fill(-4); map.revision++;
  s.nav.setDestination({ x: s.pose.x, z: -2.35 }); s.nav.start();
  let edge = null;
  for (let i = 0; i < 600; i++) {
    const result = s.step();
    assert.ok(huellaTransitable(s.world, s.pose));
    if (result.groundStop) {
      assert.equal(result.stopped, false);
      edge = s.pose;
      break;
    }
    assert.equal(result.stopped, false);
  }
  assert.ok(edge, 'Debe detectar el límite real del suelo');
  s.step();
  assert.equal(s.nav.state, 'retroceso');
  assert.equal(s.lastCommand.back, true);
  for (let i = 0; i < 18; i++) {
    s.step();
    assert.ok(huellaTransitable(s.world, s.pose));
  }
  assert.ok(s.pose.z > edge.z + .2, 'Debe salir del borde por donde vino');
  assert.equal(s.nav.map, map);
  assert.equal(s.diagnostics.contacts, 0);
  assert.equal(map.filter(v => v === OCCUPIED).length, 0, 'Un límite de suelo no es una pared');
});

test('el límite de suelo manual detiene el avance y permite retroceder desde la misma posición', () => {
  const s = room();
  let edge = null;
  for (let i = 0; i < 180; i++) {
    const result = s.step({ forward: true });
    if (result.groundStop) { assert.equal(result.stopped, true); edge = s.pose; break; }
  }
  assert.ok(edge);
  assert.equal(s.nav.state, 'pausado');
  for (let i = 0; i < 18; i++) s.step({ back: true });
  assert.ok(s.pose.z > edge.z + .25);
  assert.equal(s.diagnostics.contacts, 0);
  assert.ok(huellaTransitable(s.world, s.pose));
});

for (const noisy of [false, true]) test(`elige una ruta y se retira de una pared tras contacto real (ruido: ${noisy})`, () => {
  const s = createScanSession({
    triangles: new Float32Array([
      -2, 0, -2, 2, 0, -2, 2, 0, 2, -2, 0, -2, 2, 0, 2, -2, 0, 2,
      -2, 0, -1.6, 2, 0, -1.6, 2, 1, -1.6, -2, 0, -1.6, 2, 1, -1.6, -2, 1, -1.6,
    ]),
    bounds: { minX: -2, maxX: 2, minZ: -2, maxZ: 2, maxY: 1 },
  }, { noisy, seed: 7 });
  const base = s.pose, map = s.nav.map;
  for (let i = 0; i < 240 && !s.diagnostics.contacts; i++) s.step({ forward: true });
  assert.equal(s.diagnostics.contacts, 1);
  const contactPose = s.pose;
  // This is the same selector the map click uses: no artificial free map,
  // forced state or fabricated bumper measurement.
  const selected = selectMapDestination(s.nav, base, s.world.radius);
  let reverseSteps = 0;
  for (let i = 0; i < 600; i++) {
    const result = s.step();
    if (s.lastCommand.back) reverseSteps++;
    assert.ok(huellaTransitable(s.world, s.pose));
    if (result.stopped) break;
  }
  assert.ok(reverseSteps > 0, 'Debe ejecutar el retroceso físicamente');
  assert.ok(s.pose.z > contactPose.z + .3);
  assert.equal(s.nav.state, 'destino_alcanzado');
  assert.ok(Math.hypot(s.pose.x - selected.goal.x, s.pose.z - selected.goal.z) < .15);
  assert.equal(s.nav.map, map);
  assert.equal(s.diagnostics.contacts, 1, 'No debe volver a empujar la pared');
});

for (const noisy of [false, true]) for (const turnSteps of [60, 120, 180]) test(`sale de la pared con cámara girada ${turnSteps * 1.5} grados (ruido: ${noisy})`, () => {
  const s = createScanSession({
    triangles: new Float32Array([
      -2, 0, -2, 2, 0, -2, 2, 0, 2, -2, 0, -2, 2, 0, 2, -2, 0, 2,
      -2, 0, -1.6, 2, 0, -1.6, 2, 1, -1.6, -2, 0, -1.6, 2, 1, -1.6, -2, 1, -1.6,
    ]),
    bounds: { minX: -2, maxX: 2, minZ: -2, maxZ: 2, maxY: 1 },
  }, { noisy, seed: 7 });
  const base = s.pose;
  for (let i = 0; i < 240 && !s.diagnostics.contacts; i++) s.step({ forward: true });
  assert.equal(s.diagnostics.contacts, 1);
  for (let i = 0; i < turnSteps; i++) s.step({ left: true });
  const wallPose = s.pose;
  const selected = selectMapDestination(s.nav, base, s.world.radius);
  let advanced = false;
  for (let i = 0; i < 600; i++) {
    const result = s.step();
    advanced ||= Boolean(s.lastCommand.forward);
    assert.ok(huellaTransitable(s.world, s.pose));
    if (result.stopped) break;
  }
  assert.ok(advanced);
  assert.ok(s.pose.z > wallPose.z + .3);
  assert.equal(s.nav.state, 'destino_alcanzado');
  assert.ok(Math.hypot(s.pose.x - selected.goal.x, s.pose.z - selected.goal.z) < .15);
  assert.equal(s.diagnostics.contacts, 1, 'La retirada no debe empujar la pared que queda detrás');
});
