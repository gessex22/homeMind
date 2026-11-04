import test from 'node:test';
import assert from 'node:assert/strict';
import { createMap, cell, integrate, FREE, OCCUPIED } from '../src/mapping.js';
import { projectScan, SENSOR } from '../src/sensor.js';
import { alignMaps, compareMaps } from '../src/map-comparison.js';
import { trialOptions, runTrial, TRIAL_COUNT } from '../src/trials.js';
import { SENSORES_RUIDOSOS } from '../src/odometry.js';
import { SCENARIOS } from '../src/world.js';

test('proyectar ignora ángulos verdaderos y usa orientación estimada', () => {
  const raw = [{ angle: 999, kind: 'wall', distance: 1 }];
  const a = projectScan(raw, { yaw: 0 }), b = projectScan(raw, { yaw: Math.PI / 2 });
  assert.equal(a[0].angle, 0); assert.equal(b[0].angle, Math.PI / 2);
  assert.equal(raw[0].angle, 999);
  const at = { x: .05, z: .05, yaw: 0 }, first = createMap(), rotated = createMap();
  for (let i = 0; i < 2; i++) { integrate(first, at, a); integrate(rotated, at, b); }
  assert.equal(first[cell(.05, -.955)], OCCUPIED);
  assert.equal(rotated[cell(-.955, .05)], OCCUPIED);
  assert.notDeepEqual(first, rotated);
  const many = projectScan([{ angle: 1 }, { angle: 2 }, { angle: 3 }], { yaw: .2 });
  assert.equal(many[0].angle, .2 - SENSOR.fov / 2);
  assert.equal(many[2].angle, .2 + SENSOR.fov / 2);
});

function tiny(values, id = 1) {
  return { id, map: Uint8Array.from(values), grid: { originX: 0, originZ: 0, width: values.length, height: 1, resolution: .1 } };
}
test('desconocido no vota, falta evidencia requiere 3 y un desacuerdo 9 a 1 se conserva', () => {
  const trials = Array.from({ length: 10 }, (_, i) => tiny([0, i < 2 ? FREE : 0, i === 9 ? OCCUPIED : FREE, OCCUPIED], i + 1));
  const c = compareMaps(trials);
  assert.deepEqual([...c.observedVotes], [0, 2, 10, 10]);
  assert.deepEqual([...c.classes], [0, 1, 4, 3]);
  assert.equal(c.freeVotes[2], 9); assert.equal(c.occupiedVotes[2], 1);
  assert.equal(c.stats.conflictos, 1);
  assert.equal(c.stats.alineaciones_sin_confirmar, 9);
});
test('cada pasada aporta como máximo un voto aun con distintas resoluciones', () => {
  const a = tiny([FREE, 0]);
  const b = { ...tiny(Array(8).fill(OCCUPIED), 2), grid: { ...a.grid, width: 4, height: 2, resolution: .05 } };
  const c = compareMaps([a, b]);
  assert.deepEqual([...c.occupiedVotes], [1, 1]);
  assert.ok(c.observedVotes.every(n => n <= 2));
  const again = compareMaps([a, b]);
  assert.deepEqual(again.observedVotes, c.observedVotes);
});
function cornerMap(shift = 0) {
  const map = createMap({ originX: -2, originZ: -2, resolution: .1, width: 40, height: 40 });
  map.fill(FREE);
  for (let i = 0; i < 20; i++) {
    const col = 10 + Math.round(shift / .1);
    map[10 * 40 + col + i] = OCCUPIED;
    map[(10 + i) * 40 + col] = OCCUPIED;
  }
  return { id: 1, map, grid: map.grid };
}
test('alineación local recupera traslación de paredes observadas sin geometría del mundo', () => {
  const a = cornerMap(), b = cornerMap(.2), fix = alignMaps(a, b);
  assert.equal(fix.applied, true, JSON.stringify(fix));
  assert.ok(Math.abs(fix.x + .2) < .051, JSON.stringify(fix));
  assert.ok(Math.abs(fix.z) < .051);
  assert.ok(Math.abs(fix.yaw) < .041);
});
test('pared uniforme no confirma una alineación única y señala ambigüedad', () => {
  const map = createMap({ originX: -4, originZ: -4, resolution: .1, width: 80, height: 80 });
  map.fill(FREE);
  for (let i = 0; i < 80; i++) map[20 * 80 + i] = OCCUPIED;
  const t = { map, grid: map.grid }, fix = alignMaps(t, t);
  assert.equal(fix.applied, false);
  assert.match(fix.status, /ambigua/);
});
test('10 semillas únicas mantienen parámetros y cada ejecución recrea su mapa y estado', () => {
  const options = Array.from({ length: TRIAL_COUNT }, (_, i) => trialOptions(i));
  assert.equal(options.length, 10); assert.equal(new Set(options.map(o => o.seed)).size, 10);
  assert.deepEqual(options.map(o => o.scanDirection), [1, -1, 1, -1, 1, -1, 1, -1, 1, -1]);
  const trials = options.map(o => runTrial(SCENARIOS.partition, { ...o, maxSteps: 60 }));
  for (const trial of trials) {
    assert.deepEqual(trial.config.odometry, SENSORES_RUIDOSOS);
    assert.equal(trial.state, 'limite_simulacion');
  }
  const duplicate = runTrial(SCENARIOS.partition, { ...options[0], maxSteps: 60 });
  assert.deepEqual(duplicate.map, trials[0].map); assert.deepEqual(duplicate.summary, trials[0].summary);
  assert.notEqual(duplicate.map, trials[0].map);
  const original = trials[1].map.slice(); trials[0].map.fill(0);
  assert.deepEqual(trials[1].map.slice(), original);
  assert.notDeepEqual(trials[2].map.slice(), trials[4].map.slice());
});
