import test from 'node:test';
import assert from 'node:assert/strict';
import { parseVision, analyzeImage, readEvents, responseText } from '../server/meta-ai.js';
import { groundDetection, approachForObject, faceObjectCommand } from '../src/ai-grounding.js';
import { createNavigator } from '../src/navigation.js';
import { cell, FREE, OCCUPIED } from '../src/mapping.js';
import { footprintClear } from '../src/planner.js';

test('Muse debe indicar presencia y un concepto válido; rechaza salidas parciales o inventadas', () => {
  assert.deepEqual(parseVision('{"found":false,"description":"No hay una cama visible"}'), { found: false, target: null, description: 'No hay una cama visible' });
  assert.equal(parseVision('```json\n{"found":true,"target":"office chair","description":"silla"}\n```').target, 'office chair');
  assert.throws(() => parseVision('texto sin JSON'));
  assert.throws(() => parseVision('{"found":true,"description":"silla"}'));
  assert.equal(responseText({ output: [{ type: 'reasoning', encrypted_content: 'private' }, { type: 'message', content: [{ type: 'output_text', text: 'observación' }] }] }), 'observación');
});
test('un objeto ausente no llama a SAM ni crea una detección; la credencial solo viaja a Meta', async () => {
  let calls = 0;
  const result = await analyzeImage({ image: 'data:image/jpeg;base64,AA==', width: 640, height: 360, instruction: 'busca un cargador' }, { key: 'test-secret', fetchImpl: async (url, request) => {
    calls++; assert.equal(url, 'https://api.meta.ai/v1/responses'); assert.equal(request.headers.Authorization, 'Bearer test-secret');
    return Response.json({ status: 'completed', id: 'test-response', model: 'muse-spark-1.3', output_text: '{"found":false,"description":"No visible"}' });
  } });
  assert.equal(calls, 1); assert.deepEqual(result.detections, []); assert.equal(result.trace.segmentation, null);
  assert.ok(!JSON.stringify(result).includes('test-secret'));
});
test('validación y errores de la API no transmiten claves ni activan respuestas de prueba', async () => {
  const body = { image: 'data:image/jpeg;base64,AA==', width: 640, height: 360, instruction: 'silla' };
  await assert.rejects(analyzeImage(body, {}), /API key/);
  await assert.rejects(analyzeImage({ ...body, image: 'https://untrusted.example/a.jpg' }, { key: 'test' }), /captura/);
  await assert.rejects(analyzeImage(body, { key: 'test', fetchImpl: async () => new Response('private-upstream-error', { status: 401 }) }), error => /HTTP 401/.test(error.message) && !error.message.includes('private-upstream'));
});
test('el lector SAM tolera eventos cortados entre chunks y separadores CRLF', async () => {
  const chunks = ['data: {"type":"response.output_text.delta","delta":"mask"}\r', '\n\r', '\ndata: {"type":"response.completed"}\r\n\r\n'];
  const response = { body: (async function* () { for (const c of chunks) yield new TextEncoder().encode(c); })() };
  const events = []; for await (const event of readEvents(response)) events.push(event);
  assert.equal(events.length, 2); assert.equal(events[0].delta, 'mask');
});
test('la profundidad se transforma con la pose estimada; descarta pocos retornos y valores inválidos', () => {
  const points = Array.from({ length: 10 }, () => ({ x: 0, y: -.1, z: -2 }));
  const p = groundDetection(points, { x: 5, z: 3, yaw: Math.PI / 2 });
  assert.ok(Math.abs(p.x - 2.89) < 1e-8); assert.ok(Math.abs(p.z - 3) < 1e-8); assert.equal(p.y, .5);
  assert.equal(groundDetection(points.slice(0, 4), { x: 0, z: 0, yaw: 0 }), null);
  assert.equal(groundDetection(points.map(p => ({ ...p, z: -10 })), { x: 0, z: 0, yaw: 0 }), null);
});
test('la aproximación usa suelo conocido y deja separación al objeto', () => {
  const nav = createNavigator({ originX: -2, originZ: -2, resolution: .1, width: 40, height: 40 }, { noisy: false, footprint: .1 });
  nav.map.fill(FREE); nav.map[cell(0, -1, nav.map)] = OCCUPIED; nav.map.revision++;
  const goal = approachForObject(nav, { x: 0, z: -1 }, .1);
  assert.ok(goal); assert.ok(footprintClear(nav.map, goal, .1)); assert.ok(Math.hypot(goal.x, goal.z + 1) >= .55);
  nav.map.fill(0); nav.map.revision++;
  assert.equal(approachForObject(nav, { x: 0, z: -1 }, .1), null);
});
test('el giro final mira al punto observado y toma el ángulo corto', () => {
  assert.equal(faceObjectCommand({ x: 0, z: 0, yaw: 0 }, { x: 0, z: -1 }), null);
  assert.equal(faceObjectCommand({ x: 0, z: 0, yaw: 0 }, { x: 1, z: 0 }).right, true);
  assert.equal(faceObjectCommand({ x: 0, z: 0, yaw: Math.PI - .1 }, { x: .1, z: 1 }).left, true);
});
