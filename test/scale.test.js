import test from 'node:test';
import assert from 'node:assert/strict';
import { SCENARIOS } from '../src/world.js';
import { sense, SENSOR } from '../src/sensor.js';
import { createMap, integrate, seedFootprint, cell, center, gridOf, FREE, OCCUPIED, UNKNOWN } from '../src/mapping.js';
import { plan } from '../src/planner.js';
import { evaluateScenario } from '../scripts/evaluate-room.mjs';

test('los mapas de distinto tamaño mantienen sus coordenadas independientes', () => {
  const small = createMap(), large = createMap(SCENARIOS.large.mapConfig);
  assert.equal(small.length, 6400);
  assert.equal(large.length, 40000);
  assert.equal(cell(5, 6, small), -1);
  assert.ok(cell(5, 6, large) >= 0);
  const p = center(cell(5.05, 6.05, large), large);
  assert.ok(Math.abs(p.x - 5.05) < 1e-8 && Math.abs(p.z - 6.05) < 1e-8);
  assert.equal(gridOf(small).originX, -4);
  assert.equal(gridOf(large).originX, -10);
});

test('la lectura válida de suelo permite explorar sin una pared cercana', () => {
  const pose = { x: 0, z: 3, yaw: Math.PI }, map = createMap(SCENARIOS.large.mapConfig);
  const rays = sense(pose, SCENARIOS.large.walls);
  assert.ok(rays.every(ray => ray.kind === 'floor' && Number.isFinite(ray.distance)));
  integrate(map, pose, rays);
  assert.equal(map[cell(0, 5, map)], FREE);
  assert.notEqual(map[cell(0, 6.8, map)], OCCUPIED);
  assert.equal(map[cell(0, 7.2, map)], UNKNOWN);
  assert.equal(map[cell(0, 1, map)], UNKNOWN);
});

test('un retorno inválido no equivale a suelo libre y la pared mantiene su oclusión', () => {
  const pose = { x: 0, z: 0, yaw: 0 }, map = createMap();
  integrate(map, pose, [{ angle: 0, distance: 3, kind: 'invalid' }, { angle: 0, distance: null }]);
  assert.equal(map[cell(0, -1, map)], UNKNOWN);
  integrate(map, pose, sense(pose, [[-3, -1, 3, -1]], { ...SENSOR, rays: 1 }));
  assert.equal(map[cell(0, -.5, map)], FREE);
  assert.equal(map[cell(0, -1.5, map)], UNKNOWN);
  const broken = sense({ ...pose, z: 3, yaw: Math.PI }, SCENARIOS.large.walls, { ...SENSOR, ground: false });
  assert.ok(broken.every(ray => ray.kind === 'invalid' && ray.distance === null));
});

test('alcanzar el borde del dominio de trabajo no equivale a completar una habitación', () => {
  const map = createMap({ originX: -1, originZ: -1, resolution: .1, width: 20, height: 20 });
  map.fill(FREE);
  const scanned = [];
  for (let i = 0; i < map.length; i++) scanned.push(center(i, map));
  const result = plan(map, { x: 0, z: 0 }, scanned);
  assert.equal(result.reason, 'limitado');
  assert.ok(result.fronts.length > 0);
});

test('una vista inicial no precarga la sala grande ni las zonas tras la pared', () => {
  const map = createMap(SCENARIOS.large.mapConfig), pose = { x: 0, z: 0, yaw: 0 };
  seedFootprint(map, pose);
  integrate(map, pose, sense(pose, SCENARIOS.large.walls));
  assert.equal(map[cell(0, -3, map)], UNKNOWN);
  assert.equal(map[cell(5, 7, map)], UNKNOWN);
});

test('sala grande se descubre por recorrido manteniendo el alcance del sensor', () => {
  const { report } = evaluateScenario(SCENARIOS.large);
  assert.equal(report.aprobado, true, JSON.stringify(report));
  assert.equal(report.alcance_sensor_m, 3.8);
  assert.ok(report.recorrido_metros > 12);
  assert.ok(report.celdas_observadas_fuera_del_alcance_inicial > 1000);
  assert.equal(report.fronteras_pendientes, 0);
  assert.equal(report.colisiones, 0);
  assert.equal(report.pasos_con_huella_fuera_del_espacio_libre_conocido, 0);
});
