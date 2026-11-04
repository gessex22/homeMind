import test from 'node:test';
import assert from 'node:assert/strict';
import { createMap, FREE, OCCUPIED, UNKNOWN, center } from '../src/mapping.js';
import { routeTo, traversable, footprintClear } from '../src/planner.js';
import { createExplorer, exploreStep } from '../src/explorer.js';
import { predictMotion, STEP } from '../src/motion.js';

// Sala amplia con un corredor de 0,3 m (más angosto que la huella de 0,5 m)
// que termina en zona desconocida. La frontera del fondo del corredor y
// cualquier destino allí quedan fuera del alcance de la huella completa:
// el planificador no puede trazar la ruta entera, pero sí puede avanzar
// la parte libre hasta la boca del corredor y observar desde ahí.
function corredorAngosto() {
  const map = createMap({ originX: -2, originZ: -2, resolution: .1, width: 40, height: 40 });
  map.fill(OCCUPIED);
  for (let row = 24; row <= 38; row++) for (let col = 5; col <= 34; col++) map[row * 40 + col] = FREE;
  for (let row = 4; row <= 23; row++) for (let col = 19; col <= 21; col++) map[row * 40 + col] = FREE;
  for (let row = 1; row <= 3; row++) for (let col = 19; col <= 21; col++) map[row * 40 + col] = UNKNOWN;
  return map;
}
const inicio = { x: .05, z: 1.25, yaw: 0 }; // centro de la sala, mirando al corredor (-z)
const fondo = { x: .05, z: -1.55 }; // celda libre del fondo del corredor, tras la huella

test('routeTo con parcial llega a la celda segura más cercana; sin parcial sigue siendo null', () => {
  const map = corredorAngosto();
  assert.equal(routeTo(map, inicio, fondo, { radius: .25 }), null, 'el contrato estricto no cambia');
  const route = routeTo(map, inicio, fondo, { radius: .25, parcial: true });
  assert.ok(route?.length > 0, 'con parcial debe avanzar la parte libre');
  const safe = traversable(map, .25 * .8);
  for (const i of route) assert.equal(safe[i], 1);
  const fin = center(route.at(-1), map);
  assert.ok(fin.z > .4, `no debe entrar en el corredor: z=${fin.z}`);
  assert.ok(Math.hypot(fin.x - fondo.x, fin.z - fondo.z) < Math.hypot(inicio.x - fondo.x, inicio.z - fondo.z) - .3);
  const desdeBoca = { ...fin, yaw: 0 };
  // Replanificar desde la boca puede acercar un paso más con el margen de
  // huella; la convergencia debe terminar en null, sin ciclos.
  let pose = desdeBoca, siguiente = routeTo(map, pose, fondo, { radius: .25, parcial: true });
  for (let n = 0; siguiente && n < 10; n++) {
    const destinoParcial = center(siguiente.at(-1), map);
    assert.ok(Math.hypot(destinoParcial.x - fondo.x, destinoParcial.z - fondo.z)
      < Math.hypot(pose.x - fondo.x, pose.z - fondo.z), 'cada acercamiento parcial progresa');
    pose = { ...destinoParcial, yaw: 0 };
    siguiente = routeTo(map, pose, fondo, { radius: .25, parcial: true });
  }
  assert.equal(siguiente, null, 'sin progreso posible desde la boca, parcial también es null');
});

test('routeTo parcial no persigue un destino todavía no descubierto', () => {
  const map = corredorAngosto();
  const oculto = { x: .05, z: -1.85 }; // celdas UNKNOWN al fondo del corredor
  map[3 * 40 + 20] = UNKNOWN; map[2 * 40 + 20] = UNKNOWN; map[1 * 40 + 20] = UNKNOWN;
  assert.equal(routeTo(map, inicio, oculto, { radius: .25, parcial: true }), null);
});

test('el explorador avanza hacia un destino tras el corredor en vez de no moverse', () => {
  const map = corredorAngosto();
  const e = createExplorer(.25);
  e.state = 'planificando'; e.destino = { ...fondo };
  let pose = { ...inicio }, recorrido = 0;
  for (let i = 0; i < 900; i++) {
    const cmd = exploreStep(e, map, pose, STEP);
    const next = predictMotion(pose, cmd, STEP);
    assert.ok(footprintClear(map, next, e.footprint), `huella fuera de espacio libre en el paso ${i}`);
    recorrido += Math.hypot(next.x - pose.x, next.z - pose.z);
    pose = next;
    if (['limitado', 'bloqueado', 'destino_alcanzado'].includes(e.state)) break;
  }
  assert.notEqual(e.state, 'destino_alcanzado');
  assert.ok(recorrido > .3, `debió avanzar la parte libre, recorrió ${recorrido.toFixed(2)} m`);
  assert.ok(Math.hypot(pose.x - fondo.x, pose.z - fondo.z) < Math.hypot(inicio.x - fondo.x, inicio.z - fondo.z) - .3,
    `debió quedar al menos 0,3 m más cerca del destino: ${JSON.stringify(pose)}`);
});
