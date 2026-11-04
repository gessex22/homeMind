import test from 'node:test';
import assert from 'node:assert/strict';
import { SCENARIOS, solidos } from '../src/world.js';
import { sense, SENSOR } from '../src/sensor.js';
import { advance } from '../src/physics.js';
import { createMap, integrate, cell, FREE } from '../src/mapping.js';
import { evaluateScenario } from '../scripts/evaluate-room.mjs';

const desorden = SCENARIOS.desorden;

test('las patas se ven y los cacharros bajos son invisibles al lidar', () => {
  const pata = sense({ x: 2.1, z: .5, yaw: 0 }, desorden.walls, { ...SENSOR, rays: 1 })[0];
  assert.equal(pata.kind, 'wall');
  assert.ok(Math.abs(pata.distance - 1.2) < 1e-9);
  const caja = sense({ x: -2.5, z: .5, yaw: 0 }, desorden.walls, { ...SENSOR, rays: 1 })[0];
  assert.equal(caja.kind, 'floor');
  const map = createMap(desorden.mapConfig);
  integrate(map, { x: -2.5, z: .5, yaw: 0 }, sense({ x: -2.5, z: .5, yaw: 0 }, desorden.walls));
  assert.equal(map[cell(-2.5, -.9, map)], FREE);
});

test('patas y cacharros bloquean el cuerpo aunque no se vean', () => {
  const ft = solidos(desorden);
  const pata = advance({ x: 2.1, z: .5, yaw: 0 }, { forward: true }, 2, ft);
  assert.equal(pata.collision, true);
  assert.ok(pata.z > -.7);
  const caja = advance({ x: -2.5, z: 0, yaw: 0 }, { forward: true }, 2, ft);
  assert.equal(caja.collision, true);
  assert.ok(caja.z > -.8);
});

test('las puertas siguen transitables con desorden', () => {
  const sala = advance({ x: 0, z: .5, yaw: Math.PI / 2 }, { forward: true }, 2, solidos(desorden));
  assert.equal(sala.collision, false);
  assert.ok(sala.x < -1);
  const cuarto = advance({ x: -2.5, z: -.4, yaw: Math.PI }, { forward: true }, 1, solidos(desorden));
  assert.equal(cuarto.collision, false);
  assert.ok(cuarto.z > 0);
});

test('el desorden provoca topes pero se completa con reflejo', { timeout: 120000 }, () => {
  const { report } = evaluateScenario(desorden);
  assert.equal(report.estado, 'completado', JSON.stringify(report));
  assert.equal(report.aprobado, true, JSON.stringify(report));
  assert.ok(report.colisiones >= 1);
  assert.equal(report.pasos_con_huella_fuera_del_espacio_libre_conocido, 0);
  assert.ok(report.suelo_observado_porcentaje >= 95);
  assert.ok(report.pared_observada_porcentaje >= 95);
});

test('el desorden con ruido conserva contacto y termina seguro', { timeout: 180000 }, () => {
  const { report } = evaluateScenario(desorden, 24000, { odometry: {}, seed: 7 });
  assert.ok(['completado', 'bloqueado', 'limitado'].includes(report.estado), JSON.stringify(report));
  assert.ok(report.fusion_movimiento.contacts >= 1 && report.fusion_movimiento.suppressedM > 0);
  assert.equal(report.aprobado, true, JSON.stringify(report));
  assert.ok(report.colisiones >= 1);
  assert.equal(report.pasos_con_huella_fuera_del_espacio_libre_conocido, 0);
  assert.ok(report.suelo_observado_porcentaje > 0);
});
