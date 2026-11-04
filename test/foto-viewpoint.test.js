import test from 'node:test';
import assert from 'node:assert/strict';
import { createMap, cell, FREE, OCCUPIED } from '../src/mapping.js';
import { footprintClear } from '../src/planner.js';
import { proyeccionFrontal, proyeccionCorredor, elegirPuntoDeFoto, ANCHO_VEHICULO } from '../src/foto-viewpoint.js';

function pintar(map, x0, x1, z0, z1, valor) {
  for (let x = x0; x <= x1; x += .05) for (let z = z0; z <= z1; z += .05) {
    const i = cell(x, z, map);
    if (i >= 0) map[i] = valor;
  }
}
// Free room, frontier cell at the objective, wall just behind it.
function sala() {
  const map = createMap();
  pintar(map, -3.2, 1.45, -1.2, 1.2, FREE);
  pintar(map, 1.7, 1.9, -1.2, 1.2, OCCUPIED);
  return map;
}

test('proyección frontal mide el paso libre hasta la pared', () => {
  const map = sala();
  const p = proyeccionFrontal(map, { x: -2, z: 0 }, { x: 1, z: 0 }, { maxDist: 5 });
  assert.ok(Math.abs(p.distanciaLibre - 3.7) <= .15, `distanciaLibre=${p.distanciaLibre}`);
  const libre = proyeccionFrontal(map, { x: -2, z: 0 }, { x: 0, z: 1 }, { maxDist: 1 });
  assert.equal(libre.distanciaLibre, 1);
});

test('punto de foto: se retira hasta tener paso libre y foto legible', () => {
  const map = sala();
  const punto = elegirPuntoDeFoto(map, { x: -2, z: 0 }, { x: 1.5, z: 0 });
  assert.equal(punto.modo, 'proyectado');
  assert.ok(punto.distancia >= 1.5, `distancia=${punto.distancia}`);
  assert.ok(footprintClear(map, punto, .18));
  assert.ok(punto.distanciaLibre >= punto.distancia);
});

test('sin paso frontal (mueble en el medio) cae a la posición del elemento', () => {
  const map = sala();
  pintar(map, .55, 1.3, -.45, .45, OCCUPIED);
  const punto = elegirPuntoDeFoto(map, { x: -2, z: 0 }, { x: 1.5, z: 0 });
  assert.equal(punto.modo, 'desde-elemento');
  assert.equal(punto.x, -2);
  assert.equal(punto.z, 0);
});

test('corredor de avance: solo el ancho del vehículo (0,20 m), no el cono', () => {
  assert.equal(ANCHO_VEHICULO, .2);
  const map = sala();
  // Obstacle 0,25 m off-axis: outside the 0,20 m corridor, must not block.
  pintar(map, -.2, .4, .25, .55, OCCUPIED);
  const libre = proyeccionCorredor(map, { x: -2, z: 0 }, { x: 1, z: 0 }, { maxDist: 3 });
  assert.equal(libre.hayPaso, true, JSON.stringify(libre));
  // Same obstacle at 0,05 m off-axis: inside the swept width, blocks.
  const map2 = sala();
  pintar(map2, -.2, .4, .05, .4, OCCUPIED);
  const bloqueado = proyeccionCorredor(map2, { x: -2, z: 0 }, { x: 1, z: 0 }, { maxDist: 3 });
  assert.equal(bloqueado.hayPaso, false);
  assert.ok(bloqueado.interferencia);
});

test('un hueco más angosto que el vehículo no es ruta aunque pase el rayo central', () => {
  const map = createMap();
  pintar(map, -3.2, 1.45, -1.2, 1.2, FREE);
  pintar(map, 1.7, 1.9, -1.2, 1.2, OCCUPIED);
  // Wall across the advance with a 0,15 m slot on-axis at x=0.
  pintar(map, -.05, .05, -1.2, 1.2, OCCUPIED);
  pintar(map, -.05, .05, -.075, .075, FREE);
  const rayo = proyeccionFrontal(map, { x: -2, z: 0 }, { x: 1, z: 0 }, { maxDist: 3.5 });
  assert.ok(rayo.distanciaLibre > 2, 'el rayo central sí pasa por el hueco');
  const corredor = proyeccionCorredor(map, { x: -2, z: 0 }, { x: 1, z: 0 }, { maxDist: 3.5 });
  assert.equal(corredor.hayPaso, false, 'el corredor de 0,20 m no cabe por el hueco de 0,15 m');
  const punto = elegirPuntoDeFoto(map, { x: -2, z: 0 }, { x: 1.5, z: 0 });
  assert.ok(punto.x < -.05, `ningún punto aceptado cruza el hueco: x=${punto.x}`);
  if (punto.modo === 'proyectado') {
    const avance = proyeccionCorredor(map, { x: -2, z: 0 }, { x: 1, z: 0 }, { maxDist: Math.hypot(punto.x + 2, punto.z) });
    assert.equal(avance.hayPaso, true, 'el punto aceptado sí es alcanzable por el corredor');
  }
});

test('la interferencia se localiza: el mueble que corta la proyección', () => {
  const map = sala();
  pintar(map, .55, 1.3, -.45, .45, OCCUPIED);
  const punto = elegirPuntoDeFoto(map, { x: -2, z: 0 }, { x: 1.5, z: 0 });
  assert.equal(punto.modo, 'desde-elemento');
  assert.ok(punto.interferencia, 'debe señalar el objeto que interfiere');
  assert.ok(Math.abs(punto.interferencia.x - .55) <= .2, `x=${punto.interferencia.x}`);
  const despejado = elegirPuntoDeFoto(sala(), { x: -2, z: 0 }, { x: 1.5, z: 0 });
  assert.equal(despejado.interferencia, null);
});

test('pegado a la zona no sirve: exige distancia mínima de encuadre', () => {
  const map = createMap();
  pintar(map, .9, 1.45, -1.2, 1.2, FREE);
  pintar(map, 1.7, 1.9, -1.2, 1.2, OCCUPIED);
  // Element already stands 0.5 m from the zone: no candidate fits, so
  // the photo is taken from the element instead of centimeters away.
  const punto = elegirPuntoDeFoto(map, { x: 1, z: 0 }, { x: 1.5, z: 0 });
  assert.equal(punto.modo, 'desde-elemento');
});
