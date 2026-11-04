import test from 'node:test';
import assert from 'node:assert/strict';
import { evaluatePerception } from '../src/perception-evaluation.js';

const board = { label: 'rojo', x: 0, y: .35, z: -2, yaw: 0, size: .5 };
function frame() {
  return {
    pose: { x: 0, z: 0, yaw: 0 }, tiempo_s: 1,
    imagen_png: 'data:image/png;base64,fixture',
    calibracion: { width: 480, height: 270, fx: 280, fy: 280, cx: 240, cy: 135 },
    detections: [{ label: 'rojo', frontal: true, clipped: false, bbox: { x: 205, y: 100, width: 70, height: 70 } }],
  };
}

test('la evaluación exige una detección en imagen que coincida con la proyección frontal', () => {
  assert.equal(evaluatePerception([board], [frame()]).aprobado, true);
  assert.equal(evaluatePerception([board], []).aprobado, false);
  const wrong = frame(); wrong.detections[0].bbox.x = 20;
  assert.equal(evaluatePerception([board], [wrong]).aprobado, false);
});

test('una vista rasante o recortada no aprueba por conocer la verdad del simulador', () => {
  const grazing = frame(); grazing.pose.x = 2;
  assert.equal(evaluatePerception([board], [grazing]).aprobado, false);
  const clipped = frame(); clipped.detections[0].clipped = true;
  assert.equal(evaluatePerception([board], [clipped]).aprobado, false);
  const missingPixels = frame(); delete missingPixels.imagen_png;
  assert.equal(evaluatePerception([board], [missingPixels]).aprobado, false);
});

test('dos tableros del mismo color no se fusionan en un inventario aparente', () => {
  const report = evaluatePerception([board, { ...board, x: 1 }], [frame()]);
  assert.equal(report.aprobado, false);
  assert.deepEqual(report.ambiguos, ['rojo']);
  assert.equal(report.encontrados, 0);
});

test('una vista frontal pequeña queda pendiente y se elige la mejor captura posterior', () => {
  const far = frame(); far.detections[0].bbox = { x: 225, y: 120, width: 30, height: 30 };
  const report = evaluatePerception([board], [far]);
  assert.equal(report.aprobado, false); assert.equal(report.encontrados, 1); assert.equal(report.conDetalle, 0);
  assert.equal(report.tableros[0].lado_min_px, 30); assert.equal(report.tableros[0].detalle_suficiente, false);
  const near = frame(); near.tiempo_s = 2;
  const improved = evaluatePerception([board], [far, near]);
  assert.equal(improved.aprobado, true); assert.equal(improved.tableros[0].keyframe, 2); assert.equal(improved.tableros[0].lado_min_px, 70);
  assert.equal(evaluatePerception([board], [far, near], { minLadoReferencia: 80 }).aprobado, false);
});
