import test from 'node:test';
import assert from 'node:assert/strict';
import { SCENARIOS } from '../src/world.js';
import { advance } from '../src/physics.js';
import { evaluateScenario } from '../scripts/evaluate-room.mjs';

test('casa tiene interior no regular con puertas transitables', () => {
  const casa = SCENARIOS.casa;
  assert.ok(casa);
  assert.ok(casa.walls.length > 6);
  assert.equal(casa.bounds.maxX - casa.bounds.minX, 10);
  assert.equal(casa.bounds.maxZ - casa.bounds.minZ, 8);
  // Puerta entre sala y cuartos (x=-1, z 0..1): pasar de este a oeste.
  const puertaSala = advance({ x: 0, z: 0.5, yaw: Math.PI / 2 }, { forward: true }, 2, casa.walls);
  assert.equal(puertaSala.collision, false);
  assert.ok(puertaSala.x < -1);
  // Puerta entre recámaras (z=0, x -3..-2): pasar de sur a norte.
  const puertaCuartos = advance({ x: -2.5, z: -0.5, yaw: Math.PI }, { forward: true }, 2, casa.walls);
  assert.equal(puertaCuartos.collision, false);
  assert.ok(puertaCuartos.z > 0);
  // Los tableros están en cuartos distintos.
  const cuartos = casa.boards.map(b => `${b.x < -1 ? 'oeste' : b.x > 3 && b.z > 2 ? 'baño' : 'sala'}:${b.z < 0 ? 'sur' : 'norte'}`);
  assert.equal(new Set(cuartos).size, 3);
});

test('casa se explora completa sin colisiones', { timeout: 180000 }, () => {
  const { report } = evaluateScenario(SCENARIOS.casa);
  assert.equal(report.aprobado, true, JSON.stringify(report));
  assert.equal(report.estado, 'completado');
  assert.equal(report.colisiones, 0);
  assert.equal(report.pasos_con_huella_fuera_del_espacio_libre_conocido, 0);
});
