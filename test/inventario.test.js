import test from 'node:test';
import assert from 'node:assert/strict';
import { fuseInventory, locateDetection, RADIO_FUSION_M } from '../src/inventory.js';

const CALIB = { width: 480, height: 270, fx: 286, fy: 286, cx: 240, cy: 135 };

function detection(pose, board, { label = 'rojo', frontal = true } = {}) {
  const dx = board.x - pose.x, dz = board.z - pose.z;
  const dist = Math.hypot(dx, dz);
  const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
  const beta = Math.atan2(dx * c - dz * s, -dx * s - dz * c);
  const narrow = frontal ? 1 : (pose.z - board.z) / dist;
  const w = Math.max(1, Math.round(286 * .48 / dist * narrow));
  const u = 240 + 286 * Math.tan(beta);
  return { label, bbox: { x: u - w / 2, y: 135 - w / 2, width: w, height: w }, area: w * w, frontal, clipped: false };
}

function frame(id, pose, specs) {
  return { id, pose, tiempo_s: id, calibracion: CALIB, detections: specs.map(s => detection(s.real ?? pose, s.board, s)) };
}

const face = (pose, board) => Math.atan2(-(board.x - pose.x), -(board.z - pose.z));

test('tres vistas del mismo tablero fusionan en una entrada', () => {
  const board = { x: 0, z: -3 };
  const p1 = { x: 0, z: 0, yaw: 0 };
  const p2 = { x: 1, z: -1, yaw: face({ x: 1, z: -1 }, board) };
  const p3 = { x: 1.5, z: 0, yaw: face({ x: 1.5, z: 0 }, board) };
  const frames = [
    frame(1, p1, [{ board }]),
    frame(2, p2, [{ board }]),
    frame(3, p3, [{ board, frontal: false }]),
  ];
  const [entry] = fuseInventory(frames);
  assert.equal(fuseInventory(frames).length, 1);
  assert.equal(entry.id, 'rojo-1');
  assert.equal(entry.observaciones, 3);
  assert.equal(entry.frontales, 2);
  assert.deepEqual(entry.keyframes, [1, 2, 3]);
  assert.equal(entry.primera_vista_s, 1);
  assert.equal(entry.ultima_vista_s, 3);
  assert.equal(entry.mejor_keyframe, 2);
  assert.ok(Math.hypot(entry.x - board.x, entry.z - board.z) < .4);
});

test('dos tableros rojos separados no se fusionan', () => {
  const a = { x: 0, z: -3 }, b = { x: 3, z: -3 };
  const frames = [frame(1, { x: 0, z: 0, yaw: 0 }, [{ board: a }]), frame(2, { x: 3, z: 0, yaw: 0 }, [{ board: b }])];
  const entries = fuseInventory(frames);
  assert.equal(entries.length, 2);
  assert.deepEqual(entries.map(e => e.id), ['rojo-1', 'rojo-2']);
  assert.ok(Math.hypot(entries[0].x - a.x, entries[0].z - a.z) < .3);
  assert.ok(Math.hypot(entries[1].x - b.x, entries[1].z - b.z) < .3);
});

test('tolera deriva leve entre pose creída y real', () => {
  const board = { x: 0, z: -3 };
  const real = { x: 0, z: 0, yaw: 0 };
  const frames = [
    frame(1, real, [{ board, real }]),
    frame(2, { x: .3, z: .2, yaw: .05 }, [{ board, real }]),
  ];
  const entries = fuseInventory(frames);
  assert.equal(entries.length, 1);
  assert.equal(entries[0].observaciones, 2);
  assert.ok(Math.hypot(entries[0].x - board.x, entries[0].z - board.z) < .4);
});

test('etiquetas distintas no se mezclan aunque coincidan en posición', () => {
  const frames = [frame(1, { x: 0, z: 0, yaw: 0 }, [{ board: { x: 0, z: -3 } }, { board: { x: 0, z: -3 }, label: 'azul' }])];
  const entries = fuseInventory(frames);
  assert.deepEqual(entries.map(e => e.id), ['rojo-1', 'azul-1']);
});

test('una observación localiza por marcación y tamaño aparente', () => {
  const at = locateDetection(
    { pose: { x: 0, z: 0, yaw: 0 }, calibracion: CALIB },
    { bbox: { x: 240 - 23, y: 100, width: 46, height: 46 } },
  );
  assert.ok(Math.hypot(at.x - 0, at.z + 2.99) < .05);
});

test('entradas inválidas se ignoran y el radio se valida', () => {
  assert.deepEqual(fuseInventory([]), []);
  assert.deepEqual(fuseInventory([{ id: 1, pose: { x: 0, z: 0, yaw: 0 }, detections: [{ label: 'rojo' }] }]), []);
  assert.deepEqual(fuseInventory([{ id: 1, pose: { x: 0, z: 0, yaw: 0 }, calibracion: CALIB, detections: [{ label: '', bbox: { x: 1, y: 1, width: 5, height: 5 } }] }]), []);
  assert.throws(() => fuseInventory([], { radio: 0 }), RangeError);
  assert.throws(() => fuseInventory([], { radio: -1 }), RangeError);
  assert.equal(RADIO_FUSION_M, .75);
});
