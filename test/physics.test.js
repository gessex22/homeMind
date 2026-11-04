import test from 'node:test';
import assert from 'node:assert/strict';
import { advance, sample, STEP } from '../src/physics.js';
import { predictMotion } from '../src/motion.js';
const idle = { forward: false, back: false, left: false, right: false };
test('la predicción del controlador no conoce los límites físicos del simulador', () => {
  const result = predictMotion({ x: 20, z: 20, yaw: 0 }, { ...idle, forward: true }, 1);
  assert.equal(result.x, 20);
  assert.equal(result.z, 19);
  assert.equal(result.collision, undefined);
});
test('avanza hacia -Z y gira hacia -X', () => {
  const start = { x: 0, z: 0, yaw: 0 };
  const forward = advance(start, { ...idle, forward: true }, 1);
  assert.equal(forward.z, -1); assert.equal(forward.x, 0);
  const turned = advance(start, { ...idle, left: true }, 1);
  assert.ok(Math.abs(turned.yaw - Math.PI / 2) < 1e-10);
  assert.ok(advance(turned, { ...idle, forward: true }, 1).x < -.99);
});
test('las paredes respetan el radio incluso con pasos largos', () => {
  const result = advance({ x: 0, z: 0, yaw: 0 }, { ...idle, forward: true }, 100);
  assert.equal(result.z, -2.75); assert.equal(result.collision, true);
  const reverse = advance(result, { ...idle, back: true }, 1);
  assert.equal(reverse.z, -1.75); assert.equal(reverse.collision, false);
});
test('muestras llevan pasos y marcas temporales en UTC', () => {
  const pose = { x: 1, z: 2, yaw: .5 };
  const record = sample(pose, 60, 0);
  assert.equal(record.timestamp, new Date(STEP * 60 * 1000).toISOString());
  assert.deepEqual([record.paso, record.x, record.z, record.yaw_rad], [60, 1, 2, .5]);
});
