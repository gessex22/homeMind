import test from 'node:test';
import assert from 'node:assert/strict';
import { createKeyframeSelector, selectKeyframe } from '../src/keyframes.js';

const pose = { x: 0, z: 0, yaw: 0 };

test('keyframes no duplican el reposo y acumulan movimiento desde la última captura', () => {
  const selector = createKeyframeSelector();
  assert.equal(selectKeyframe(selector, pose, 0).reason, 'inicial');
  assert.equal(selectKeyframe(selector, pose, 10).capture, false);
  for (const x of [.03, .06, .09]) assert.equal(selectKeyframe(selector, { ...pose, x }, 11).capture, false);
  assert.equal(selectKeyframe(selector, { ...pose, x: .1 }, 11).reason, 'traslacion');
  assert.equal(selector.count, 2);
});

test('keyframes mantienen solapamiento angular y no confunden el cruce de ±pi', () => {
  const selector = createKeyframeSelector();
  selectKeyframe(selector, { ...pose, yaw: 179 * Math.PI / 180 }, 0);
  assert.equal(selectKeyframe(selector, { ...pose, yaw: -179 * Math.PI / 180 }, 1).capture, false);
  assert.equal(selectKeyframe(selector, { ...pose, yaw: -171 * Math.PI / 180 }, 2).reason, 'giro');
});

test('cadencia, límite y reinicio de keyframes son explícitos', () => {
  const selector = createKeyframeSelector({ maxFrames: 2 });
  selectKeyframe(selector, pose, 0);
  assert.equal(selectKeyframe(selector, { ...pose, x: 1 }, .1).capture, false);
  assert.equal(selectKeyframe(selector, pose, .15, { force: true }).reason, 'manual');
  assert.deepEqual(selectKeyframe(selector, { ...pose, x: 2 }, 1), { capture: false, reason: 'limite' });
  assert.equal(createKeyframeSelector().count, 0);
});
