import test from 'node:test';
import * as THREE from 'three';
import assert from 'node:assert/strict';
import { groundDetection } from '../src/ai-grounding.js';
import { createNavigator } from '../src/navigation.js';
import { capturarFotoActual, crearRegistroFotosZonas } from '../src/foto-viewpoint.js';

test('camera calibration reverses pitch for varied headings using capture belief', () => {
  for (const pitchDeg of [0, 12, 30]) for (const yaw of [0, .7, -1.4]) {
    const a = pitchDeg * Math.PI / 180, z = -2.89, y = 0;
    const local = { x: .2, y: Math.cos(a) * y - Math.sin(a) * z, z: Math.sin(a) * y + Math.cos(a) * z };
    const belief = { x: 2, z: 4, yaw };
    const result = groundDetection(Array.from({ length: 7 }, () => local), belief, { pitchDeg });
    assert.ok(Math.abs(result.x - (belief.x - Math.sin(yaw) * .11 + Math.cos(yaw) * .2 + Math.sin(yaw) * z)) < 1e-9);
    assert.ok(Math.abs(result.z - (belief.z - Math.cos(yaw) * .11 - Math.sin(yaw) * .2 + Math.cos(yaw) * z)) < 1e-9);
    assert.ok(Math.abs(result.y - .6) < 1e-9);
  }
});

test('automatic relocalization pauses, resumes, and stops forward on contact', () => {
  const grid = { width: 40, height: 40, resolution: .1, originX: -2, originZ: -2 };
  const nav = createNavigator(grid, { noisy: false, priorMap: { grid, celdas: Array(1600).fill(1) } });
  nav.start(); nav.pause();
  assert.equal(nav.state, 'pausado');
  assert.equal(nav.step(.02).left, false);
  nav.start(); assert.equal(nav.step(.02).left, true);
  nav.step(.02, { rays: [], motion: { giro: 6.4, avance: 0, drive: 0, contact: false } });
  const stopped = nav.step(.02, { motion: { giro: 0, avance: .2, drive: 1, contact: true } });
  assert.equal(stopped.forward, false);
  assert.equal(nav.summary().fusion_movimiento.acceptedM, 0);
  assert.equal(nav.step(.02).forward, false);
});

test('evidence capture uses physical pose and capture belief', () => {
  const pose = { x: 9, z: 8, yaw: 1 }, belief = { x: 1, z: 2, yaw: 0 };
  const session = { pose, nav: { belief }, world: { floorY: 3 }, diagnostics: { seconds: 4 } };
  let args; const result = capturarFotoActual({ capture(...a) { args = a; return 'image'; } }, session);
  assert.equal(result, 'image'); assert.deepEqual(args, [pose, 3, belief, 4]);
});


test('grounding matches the actual Three.js camera mount and a different object height', () => {
  for (const pitchDeg of [0, 12, 30]) for (const yaw of [0, .8, -1.1]) {
    const pose = { x: 2, z: -1, yaw }, height = .6, offset = .11;
    const dx = -Math.sin(yaw), dz = -Math.cos(yaw);
    const camera = new THREE.PerspectiveCamera();
    camera.position.set(pose.x + dx * offset, height, pose.z + dz * offset);
    camera.lookAt(camera.position.x + dx, height - Math.tan(pitchDeg * Math.PI / 180), camera.position.z + dz);
    camera.updateMatrixWorld(true);
    const truth = new THREE.Vector3(pose.x + dx * 2.5, 1.15, pose.z + dz * 2.5);
    const local = camera.worldToLocal(truth.clone());
    const result = groundDetection(Array.from({ length: 6 }, () => local), pose, { pitchDeg, height, forwardOffset: offset });
    assert.ok(result && Math.hypot(result.x - truth.x, result.y - truth.y, result.z - truth.z) < 1e-9);
  }
});

test('front range blocks relocalization advance without contact', () => {
  const grid = { width: 40, height: 40, resolution: .1, originX: -2, originZ: -2 };
  const nav = createNavigator(grid, { noisy: true, priorMap: { grid, celdas: Array(1600).fill(1) } });
  nav.start(); nav.step(.02, { rays: [], motion: { giro: 6.4, avance: 0, drive: 0, contact: false } });
  const rays = Array.from({ length: 121 }, () => ({ kind: 'wall', distance: .2 }));
  assert.equal(nav.step(.02, { rays }).forward, false);
});

test('noisy contact suppresses encoder travel during relocalization', () => {
  const grid = { width: 40, height: 40, resolution: .1, originX: -2, originZ: -2 };
  const nav = createNavigator(grid, { noisy: true, priorMap: { grid, celdas: Array(1600).fill(1) } });
  nav.start(); nav.step(.02, { rays: [], motion: { giro: 6.4, avance: 0, drive: 0, contact: false } });
  nav.step(.02, { motion: { giro: 0, avance: .2, drive: 1, contact: true } });
  assert.equal(nav.summary().fusion_movimiento.acceptedM, 0);
  assert.equal(nav.summary().fusion_movimiento.encoderM, .2);
  assert.equal(nav.belief.x, 0); assert.equal(nav.belief.z, 0);
});


test('zone snapshots remain independent and preserve decisions until reset', () => {
  const store = crearRegistroFotosZonas();
  const state = { punto: { x: 1, z: 2, yaw: .3 }, belief: { x: 0, z: 0, yaw: 0 }, calibracion: { pitchDeg: 12 }, tiempo: 10, top: 'data:top1', pov: 'data:zone1' };
  let calls = 0; const capture = () => { calls++; return state; };
  const first = store.obtener([4, 2], capture);
  state.punto.x = 9; state.belief.x = 8; state.calibracion.pitchDeg = 30; state.pov = 'data:zone2'; state.tiempo = 20;
  const again = store.obtener([2, 4], capture);
  assert.deepEqual(again, first); assert.equal(calls, 1);
  const second = store.obtener([7, 8], capture);
  assert.equal(second.punto.x, 9); assert.equal(second.pov, 'data:zone2');
  assert.equal(first.calibracion.pitchDeg, 12); assert.equal(first.fuente, 'observador-proyectado'); assert.equal(first.evidenciaRobot, false);
  first.punto.x = -100; assert.equal(store.exportar()[0].punto.x, 1);
  store.decidir([4, 2], 'sin-paso'); assert.equal(store.obtener([2, 4], capture).decision, 'sin-paso');
  store.limpiar(); assert.equal(store.exportar().length, 0);
});
