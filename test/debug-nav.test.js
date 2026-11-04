import test from 'node:test';
import assert from 'node:assert/strict';
import { createMap, FREE, OCCUPIED, UNKNOWN } from '../src/mapping.js';
import { createExplorer } from '../src/explorer.js';
import { diagnosticarNavegacion } from '../src/debug-nav.js';

function corredorAngosto() {
  const map = createMap({ originX: -2, originZ: -2, resolution: .1, width: 40, height: 40 });
  map.fill(OCCUPIED);
  for (let row = 24; row <= 38; row++) for (let col = 5; col <= 34; col++) map[row * 40 + col] = FREE;
  for (let row = 4; row <= 23; row++) for (let col = 19; col <= 21; col++) map[row * 40 + col] = FREE;
  for (let row = 1; row <= 3; row++) for (let col = 19; col <= 21; col++) map[row * 40 + col] = UNKNOWN;
  return map;
}
const inicio = { x: .05, z: 1.25, yaw: 0 };

test('diagnóstico: destino descubierto sin ruta completa ofrece acercamiento parcial', () => {
  const map = corredorAngosto(), e = createExplorer(.25);
  e.state = 'planificando'; e.destino = { x: .05, z: -1.55 };
  const d = diagnosticarNavegacion({ map, creencia: inicio, explorer: e, ultimoComando: { forward: false, back: false, left: false, right: false } });
  assert.match(d.motivo, /acercamiento parcial/);
  assert.equal(d.datos.rutaDestino, null);
  assert.ok(d.datos.rutaParcial > 0);
  assert.equal(d.datos.huellaActual, true);
});

test('diagnóstico: un limitado con ruta visible se explica como recuperación agotada', () => {
  const map = corredorAngosto(), e = createExplorer(.25);
  e.state = 'limitado'; e.destino = { x: .05, z: -1.55 }; e.recoveryAttempts = 5; e.recoveryCause = 'mapa';
  const d = diagnosticarNavegacion({ map, creencia: inicio, explorer: e });
  assert.match(d.motivo, /^quedó limitado tras agotar la recuperación/);
  assert.match(d.motivo, /Replanificar ruta/);
});

test('diagnóstico: destino todavía desconocido se distingue de uno inalcanzable', () => {
  const map = corredorAngosto(), e = createExplorer(.25);
  e.state = 'planificando'; e.destino = { x: .05, z: -1.85 };
  const d = diagnosticarNavegacion({ map, creencia: inicio, explorer: e });
  assert.match(d.motivo, /aún no está descubierto/);
  assert.equal(d.datos.rutaParcial, null);
});

test('diagnóstico: huella solapada se reporta como causa del freno', () => {
  const map = corredorAngosto(), e = createExplorer(.25);
  e.state = 'destino'; e.destino = { x: .05, z: -1.55 };
  const pegado = { x: .05, z: -.35, yaw: 0 }; // dentro del corredor: la huella toca las paredes
  const d = diagnosticarNavegacion({ map, creencia: pegado, explorer: e });
  assert.equal(d.datos.huellaActual, false);
  assert.match(d.motivo, /solapa/);
});

test('diagnóstico: sin destino reporta el plan de exploración y el comando en cero', () => {
  const map = createMap({ originX: -2, originZ: -2, resolution: .1, width: 40, height: 40 });
  map.fill(FREE);
  const e = createExplorer(.25);
  e.state = 'navegando'; e.route = [20 * 40 + 21];
  const d = diagnosticarNavegacion({ map, creencia: { x: .05, z: .05, yaw: 0 }, explorer: e, ultimoComando: { forward: false, back: false, left: false, right: false } });
  assert.equal(d.datos.comandoEnCero, true);
  assert.match(d.motivo, /comando salió en cero/);
  assert.ok(d.lineas.some(l => l.startsWith('RUTA')));
});
