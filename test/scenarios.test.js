import test from 'node:test';
import assert from 'node:assert/strict';
import { SCENARIOS } from '../src/world.js';
import { createMap, FREE } from '../src/mapping.js';
import { plan } from '../src/planner.js';
import { createExplorer, startExplorer } from '../src/explorer.js';
import { evaluateScenario } from '../scripts/evaluate-room.mjs';

test('un mapa completamente desconocido no es una exploración completada', () => {
  assert.equal(plan(createMap(), { x: 0, z: 0 }).reason, 'bloqueado');
});

test('agotar posiciones ya observadas no declara completo un mapa con fronteras', () => {
  const map = createMap();
  const scanned = [];
  for (let row = 25; row < 55; row++) for (let col = 25; col < 55; col++) {
    map[row * 80 + col] = FREE;
    scanned.push({ x: -4 + (col + .5) * .1, z: -4 + (row + .5) * .1 });
  }
  const result = plan(map, { x: 0, z: 0 }, scanned);
  assert.ok(result.fronts.length > 0);
  assert.equal(result.reason, 'limitado');
});

test('una ejecución limitada se puede volver a explorar explícitamente', () => {
  const explorer = createExplorer();
  explorer.state = 'limitado';
  startExplorer(explorer);
  assert.equal(explorer.state, 'barrido');
});

test('el robot descubre la abertura transitable y explora detrás sin colisiones', { timeout: 120000 }, () => {
  const { report } = evaluateScenario(SCENARIOS.partition);
  assert.equal(report.aprobado, true, JSON.stringify(report));
  assert.equal(report.estado, 'completado');
  assert.equal(report.cruzo_abertura, true);
  assert.equal(report.colisiones, 0);
  assert.equal(report.pasos_con_huella_fuera_del_espacio_libre_conocido, 0);
});

test('ante una abertura estrecha se detiene parcialmente y conserva lo no observado', { timeout: 120000 }, () => {
  const { report } = evaluateScenario(SCENARIOS.narrow);
  assert.equal(report.aprobado, true, JSON.stringify(report));
  assert.equal(report.estado, 'limitado');
  assert.equal(report.cruzo_abertura, false);
  assert.ok(report.fronteras_pendientes > 0);
  assert.ok(report.celdas_desconocidas_detras > 0);
});
