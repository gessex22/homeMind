import test from 'node:test';
import assert from 'node:assert/strict';
import { createMap, integrate, seedFootprint, cell, UNKNOWN, FREE, OCCUPIED } from '../src/mapping.js';
import { sense, SENSOR } from '../src/sensor.js';
import { mulberry32 } from '../src/noise.js';

const pose = { x: 0, z: 1, yaw: 0 };
const hit = () => [{ angle: 0, distance: 1, kind: 'wall' }];
const freePass = () => [{ angle: 0, distance: 2, kind: 'floor' }];
const target = cell(0, -0.005);

test('un impacto aislado no confirma un muro; dos sí', () => {
  const map = createMap();
  integrate(map, pose, hit());
  assert.equal(map[target], UNKNOWN);
  integrate(map, pose, hit());
  assert.equal(map[target], OCCUPIED);
});

test('un paso libre abre espacio y dos impactos lo corrigen', () => {
  const map = createMap();
  integrate(map, pose, freePass());
  assert.equal(map[target], FREE);
  integrate(map, pose, hit());
  integrate(map, pose, hit());
  assert.equal(map[target], OCCUPIED);
});

test('los pasos rasantes no borran un muro confirmado', () => {
  const map = createMap();
  integrate(map, pose, hit());
  integrate(map, pose, hit());
  assert.equal(map[target], OCCUPIED);
  for (let i = 0; i < 50; i++) integrate(map, pose, freePass());
  assert.equal(map[target], OCCUPIED);
});

test('la huella inicial resiste impactos espurios pero cede ante muchos', () => {
  const map = createMap();
  seedFootprint(map, { x: 0, z: 0, yaw: 0 });
  const ray = [{ angle: Math.PI / 2, distance: .9, kind: 'wall' }];
  const from = { x: 1, z: 0, yaw: Math.PI / 2 };
  integrate(map, from, ray);
  assert.equal(map[cell(.095, 0, map)], FREE);
  for (let i = 0; i < 8; i++) integrate(map, from, ray);
  assert.equal(map[cell(.095, 0, map)], OCCUPIED);
});

test('barridos ruidosos repetidos confirman la pared sin sembrar lejos', () => {
  const map = createMap(), at = { x: 0, z: 0, yaw: 0 }, walls = [[-2, -1, 2, -1]];
  const random = mulberry32(11);
  for (let i = 0; i < 3; i++) integrate(map, at, sense(at, walls, { ...SENSOR, noise: .05, random }));
  assert.equal(map[cell(0, -1.005, map)], OCCUPIED);
  assert.equal(map[cell(0, -2, map)], UNKNOWN);
});
