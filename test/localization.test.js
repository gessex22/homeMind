import test from 'node:test';
import assert from 'node:assert/strict';
import { assessLocalization, createLocalizationMonitor, tickLocalization, observeLocalization } from '../src/localization.js';
import { createMap, integrate, cell, OCCUPIED } from '../src/mapping.js';
import { sense } from '../src/sensor.js';
import { SCENARIOS } from '../src/world.js';

const pose = { x: 0, z: 0, yaw: 0 };
const good = { reliable: true, reason: 'fixture_confirmada' };

test('el contador empieza al perder confirmación y no acumula tiempo localizado', () => {
  const monitor = createLocalizationMonitor();
  tickLocalization(monitor, .5);
  assert.equal(monitor.lostSeconds, 0);
  tickLocalization(monitor, .4);
  assert.equal(monitor.confirmed, false);
  assert.ok(Math.abs(monitor.lostSeconds - .3) < 1e-8);
});

test('una lectura aislada o un dato ambiguo no reinician la pérdida de localización', () => {
  const monitor = createLocalizationMonitor();
  tickLocalization(monitor, 2);
  const lost = monitor.lostSeconds;
  observeLocalization(monitor, good, pose);
  assert.equal(monitor.confirmed, false);
  assert.equal(monitor.lostSeconds, lost);
  tickLocalization(monitor, .1);
  observeLocalization(monitor, { reliable: false, reason: 'geometria_ambigua' }, pose);
  assert.ok(monitor.lostSeconds > lost);
  assert.equal(monitor.streak, 0);
});

test('la localización se recupera solo tras tres observaciones espaciadas consistentes', () => {
  const monitor = createLocalizationMonitor();
  tickLocalization(monitor, 2);
  observeLocalization(monitor, good, pose);
  for (let i = 0; i < 5; i++) observeLocalization(monitor, good, pose);
  assert.equal(monitor.streak, 1);
  tickLocalization(monitor, .1); observeLocalization(monitor, good, pose);
  assert.equal(monitor.confirmed, false);
  tickLocalization(monitor, .1); observeLocalization(monitor, good, pose);
  assert.equal(monitor.confirmed, true);
  assert.equal(monitor.lostSeconds, 0);
});

test('observaciones incompatibles entre sí no confirman una posición', () => {
  const monitor = createLocalizationMonitor();
  tickLocalization(monitor, 2); observeLocalization(monitor, good, pose);
  tickLocalization(monitor, .1); observeLocalization(monitor, good, { ...pose, x: 2 });
  assert.equal(monitor.streak, 1);
  assert.equal(monitor.confirmed, false);
});

test('suelo, lecturas inválidas y pared uniforme no bastan para localizar', () => {
  const map = createMap();
  assert.equal(assessLocalization(map, [{ kind: 'floor', distance: 2 }], pose).reliable, false);
  assert.equal(assessLocalization(map, [{ kind: 'invalid', distance: null }], pose).reliable, false);
  for (let x = -2.95; x < 3; x += .1) map[cell(x, -1.005)] = OCCUPIED;
  const rays = sense(pose, [[-3, -1, 3, -1]]);
  assert.equal(assessLocalization(map, rays, pose).reason, 'geometria_ambigua');
});

test('una esquina conocida permite confirmar sin consultar la pose verdadera', () => {
  const at = { ...pose, yaw: Math.PI / 4 }, map = createMap();
  const rays = sense(at, SCENARIOS.empty.walls);
  integrate(map, at, rays); integrate(map, at, rays);
  const result = assessLocalization(map, rays, at);
  assert.equal(result.reliable, true, JSON.stringify(result));
  const forgedAngles = rays.map(ray => ({ ...ray, angle: 99 }));
  assert.deepEqual(assessLocalization(map, forgedAngles, at), result);
});
