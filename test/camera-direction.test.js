import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { createRobotCameraSensor } from '../src/robot-camera-sensor.js';
import { predictMotion } from '../src/motion.js';
import { projectScan, SENSOR } from '../src/sensor.js';

test('la cámara RGB, el rayo central y el avance apuntan al mismo lado en todos los rumbos', () => {
  const originalDocument = globalThis.document;
  let capturedCamera;
  const renderer = {
    getRenderTarget() { return null; }, setRenderTarget() {},
    render(scene, camera) { capturedCamera = camera; }, readRenderTargetPixels() {},
  };
  // Canvas output is stubbed; the real capture method still builds the real
  // Three.js camera, so this verifies its actual optical direction.
  globalThis.document = { createElement() { return {
    getContext() { return { createImageData(w, h) { return { data: new Uint8ClampedArray(w * h * 4) }; }, putImageData() {} }; },
    toDataURL() { return 'data:image/jpeg;base64,'; },
  }; } };
  try {
    const sensor = createRobotCameraSensor({ renderer, scene: new THREE.Scene(), room: new THREE.Group(), robot: new THREE.Group(), materials: [] });
    for (const yaw of [-Math.PI, -Math.PI / 2, -.7, 0, .7, Math.PI / 2, Math.PI]) for (const pitch of [0, 12, 30]) {
      const pose = { x: .2, z: -.3, yaw };
      sensor.setPitch(pitch); sensor.capture(pose, 0, pose, 0);
      const optical = capturedCamera.getWorldDirection(new THREE.Vector3());
      const horizontalLength = Math.hypot(optical.x, optical.z);
      const next = predictMotion(pose, { forward: true }, .1);
      const [ray] = projectScan([{ kind: 'floor', distance: 1 }], pose, SENSOR);
      assert.ok(Math.abs(optical.x / horizontalLength - (next.x - pose.x) / .1) < 1e-9);
      assert.ok(Math.abs(optical.z / horizontalLength - (next.z - pose.z) / .1) < 1e-9);
      assert.ok(Math.abs(optical.x / horizontalLength + Math.sin(ray.angle)) < 1e-9);
      assert.ok(Math.abs(optical.z / horizontalLength + Math.cos(ray.angle)) < 1e-9);
    }
  } finally {
    if (originalDocument === undefined) delete globalThis.document;
    else globalThis.document = originalDocument;
  }
});
