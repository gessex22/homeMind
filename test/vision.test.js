import test from 'node:test';
import assert from 'node:assert/strict';
import { detectMarkers } from '../src/vision.js';
function image(w = 80, h = 60) { const data = new Uint8ClampedArray(w * h * 4).fill(255); return { data, width: w, height: h }; }
function paint(im, x, y, w, h, rgb) { for (let j = y; j < y + h; j++) for (let i = x; i < x + w; i++) { const p = (j * im.width + i) * 4; im.data.set([...rgb, 255], p); } }
test('blanco no produce marcadores ni clases ausentes', () => { assert.deepEqual(detectMarkers(image()), []); });
test('detecta componentes de color incluso con sombreado', () => {
  const im = image(); paint(im, 5, 5, 15, 15, [184, 25, 22]); paint(im, 30, 5, 15, 15, [20, 40, 160]); paint(im, 55, 5, 15, 15, [190, 170, 20]);
  const d = detectMarkers(im); assert.deepEqual(d.map(v => v.label).sort(), ['amarillo', 'azul', 'rojo']); assert.ok(d.every(v => v.frontal)); assert.equal(d[0].area, 225); assert.equal(d[0].lado_min_px, 15); assert.equal(d[0].proporcion_ancho_imagen, 15 / 80);
});
test('recorte, oblicuidad y oclusión no son pruebas frontales útiles', () => {
  const im = image(); paint(im, 0, 4, 15, 15, [220, 20, 20]); paint(im, 30, 5, 8, 25, [20, 30, 200]); paint(im, 50, 5, 20, 20, [230, 200, 10]); paint(im, 50, 5, 13, 10, [255, 255, 255]);
  const d = detectMarkers(im); assert.equal(d.length, 3); assert.ok(d.find(v => v.label === 'rojo').clipped); assert.ok(d.every(v => !v.frontal));
});
