import test from 'node:test';
import assert from 'node:assert/strict';
import { createMap, integrate, seedFootprint, cell, UNKNOWN, FREE, OCCUPIED, markBump } from '../src/mapping.js';
import { createExplorer, exploreStep } from '../src/explorer.js';

test('el tope pinta ocupada la zona de contacto y resiste pasos libres', () => {
  const map = createMap();
  const pose = { x: 0, z: 0, yaw: 0 };
  assert.equal(markBump(map, pose), true);
  assert.equal(map[cell(0, -.25, map)], OCCUPIED);
  integrate(map, { x: 0, z: 1, yaw: 0 }, [{ angle: 0, distance: 2, kind: 'floor' }]);
  assert.equal(map[cell(0, -.25, map)], OCCUPIED);
});

test('el tope nunca pinta la propia celda', () => {
  const fresh = createMap();
  markBump(fresh, { x: 0, z: 0, yaw: 0 });
  assert.equal(fresh[cell(0, 0, fresh)], UNKNOWN);
  const seeded = createMap();
  seedFootprint(seeded, { x: 0, z: 0, yaw: 0 });
  markBump(seeded, { x: 0, z: 0, yaw: 0 });
  assert.equal(seeded[cell(0, 0, seeded)], FREE);
});

test('el tope trasero pinta detrás', () => {
  const map = createMap();
  markBump(map, { x: 0, z: 0, yaw: 0 }, true);
  assert.equal(map[cell(0, .25, map)], OCCUPIED);
});

function bubble() {
  const map = createMap();
  for (let z = 30; z < 50; z++) for (let x = 30; x < 50; x++) map[z * 80 + x] = FREE;
  return map;
}

test('un tope navegando dispara retroceso y replanifica', () => {
  const map = bubble();
  const e = createExplorer();
  e.state = 'navegando'; e.base = { x: 0, z: 0 };
  const cmd = exploreStep(e, map, { x: 0, z: 0, yaw: 0 }, 1 / 60, true);
  assert.equal(e.state, 'retroceso');
  assert.equal(cmd.back, true);
  assert.equal(e.scanned.length, 1);
  for (let i = 0; i < 60 && e.state === 'retroceso'; i++) exploreStep(e, map, { x: 0, z: 0, yaw: 0 }, 1 / 60, false);
  assert.equal(e.state, 'planificando');
});

test('un tope sin espacio atrás pasa directo a girar', () => {
  const e = createExplorer();
  e.state = 'navegando'; e.base = { x: 0, z: 0 };
  const cmd = exploreStep(e, createMap(), { x: 0, z: 0, yaw: 0 }, 1 / 60, true);
  assert.equal(e.state, 'retroceso');
  assert.equal(e.retrocede, 30);
  assert.equal(cmd.left, true);
});
