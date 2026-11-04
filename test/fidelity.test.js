import test from 'node:test';
import assert from 'node:assert/strict';
import { SCENARIOS } from '../src/world.js';
import { advance } from '../src/physics.js';
import { sense } from '../src/sensor.js';
import { createMap, integrate, cell, UNKNOWN, FREE, OCCUPIED } from '../src/mapping.js';
const forward = { forward: true }, backward = { back: true };

test('pared interior detiene avances largos y permite retroceso', () => {
  const result = advance({ x: 0, z: 0, yaw: 0 }, forward, 10, SCENARIOS.partition.walls);
  assert.ok(result.collision); assert.ok(result.z >= -.55 - 1e-9); assert.ok(result.z <= -.54);
  const back = advance(result, backward, .2, SCENARIOS.partition.walls);
  assert.ok(back.z > result.z); assert.equal(back.collision, false);
});
test('extremos bloquean abertura estrecha y dejan pasar la abertura de un metro', () => {
  const narrow = advance({ x: 1.05, z: 0, yaw: 0 }, forward, 2, SCENARIOS.narrow.walls);
  assert.ok(narrow.collision); assert.ok(narrow.z > -.8);
  const edge = advance({ x: .85, z: 0, yaw: 0 }, forward, 2, SCENARIOS.partition.walls);
  assert.ok(edge.collision); assert.ok(edge.z > -.8);
  const through = advance({ x: 1.3, z: 0, yaw: 0 }, forward, 2, SCENARIOS.partition.walls);
  assert.equal(through.collision, false); assert.ok(through.z < -.8);
});
test('pared ocluye el mapa hasta observar desde el otro lado', () => {
  const map = createMap(), near = { x: 0, z: 0, yaw: 0 };
  integrate(map, near, sense(near, SCENARIOS.partition.walls));
  assert.equal(map[cell(0, -1.3)], UNKNOWN);
  assert.equal(map[cell(0, -.805)], OCCUPIED);
  const far = { x: 0, z: -1.2, yaw: 0 };
  integrate(map, far, sense(far, SCENARIOS.partition.walls));
  assert.equal(map[cell(0, -1.5)], FREE);
});
test('el escenario estrecho no precarga pared oculta en el mapa', () => {
  const map = createMap(), pose = { x: 0, z: 0, yaw: 0 };
  integrate(map, pose, sense(pose, SCENARIOS.narrow.walls));
  assert.equal(map[cell(0, -2.9)], UNKNOWN);
  assert.equal(map[cell(-2, -2.9)], UNKNOWN);
});
