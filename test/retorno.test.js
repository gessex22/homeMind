import test from 'node:test';
import assert from 'node:assert/strict';
import { routeTo } from '../src/planner.js';
import { createMap, cell, FREE, OCCUPIED } from '../src/mapping.js';
import { wheelSpeeds, SPEED, TURN_SPEED, predictMotion } from '../src/motion.js';
import { createExplorer, startExplorer, pauseExplorer, exploreStep, setLeash } from '../src/explorer.js';
import { tickLocalization, observeLocalization } from '../src/localization.js';
import { SCENARIOS } from '../src/world.js';
import { evaluateScenario } from '../scripts/evaluate-room.mjs';

test('routeTo une celdas libres y rechaza destinos incomunicados', () => {
  const map = createMap();
  for (let z = 20; z < 60; z++) for (let x = 20; x < 60; x++) map[z * 80 + x] = FREE;
  const route = routeTo(map, { x: 0, z: 0 }, { x: 1, z: 1 });
  assert.ok(route && route.length > 0);
  const split = createMap();
  split.fill(FREE);
  for (let x = 0; x < 80; x++) split[40 * 80 + x] = OCCUPIED;
  assert.equal(routeTo(split, { x: 0, z: -2 }, { x: 0, z: 2 }), null);
});

test('las ruedas reportan avance y giro diferenciales', () => {
  assert.deepEqual(wheelSpeeds({}), { izq: 0, der: 0, avance: 0, giro: 0 });
  const forward = wheelSpeeds({ forward: true });
  assert.equal(forward.izq, SPEED);
  assert.equal(forward.der, SPEED);
  const left = wheelSpeeds({ left: true });
  assert.ok(left.izq < 0 && left.der > 0 && left.avance === 0);
  assert.equal(left.der - left.izq, TURN_SPEED * .3);
});

test('parámetros de correa inválidos se rechazan', () => {
  const explorer = createExplorer();
  assert.throws(() => setLeash(explorer, { presupuestoS: 0, maxDistanciaM: 5 }), RangeError);
  assert.throws(() => setLeash(explorer, { presupuestoS: 30, maxDistanciaM: -1 }), RangeError);
});

test('tras refugiarse se puede volver a explorar explícitamente', () => {
  const explorer = createExplorer();
  explorer.state = 'refugio';
  startExplorer(explorer);
  assert.equal(explorer.state, 'barrido');
});

test('el tiempo total de exploración no consume el margen sin localización', { timeout: 120000 }, () => {
  const { report } = evaluateScenario(SCENARIOS.partition, 24000, { leash: { presupuestoS: 25, maxDistanciaM: 50 }, seed: 7 });
  assert.equal(report.estado, 'completado', JSON.stringify(report));
  assert.equal(report.retorno, 'no');
  assert.equal(report.colisiones, 0);
  assert.ok(report.tiempo_exploracion_s > 25);
  assert.ok(report.max_tiempo_sin_localizacion_s < 25);
});

test('límite de distancia también dispara el retorno', { timeout: 120000 }, () => {
  const { report } = evaluateScenario(SCENARIOS.partition, 24000, { leash: { presupuestoS: 600, maxDistanciaM: 1 }, seed: 7 });
  assert.equal(report.estado, 'refugio', JSON.stringify(report));
  assert.equal(report.retorno, 'base');
  assert.equal(report.colisiones, 0);
});

test('correa holgada no interfiere con la exploración completa', { timeout: 120000 }, () => {
  const { report } = evaluateScenario(SCENARIOS.partition, 24000, { leash: { presupuestoS: 600, maxDistanciaM: 50 }, seed: 7 });
  assert.equal(report.estado, 'completado');
  assert.equal(report.retorno, 'no');
  assert.equal(report.aprobado, true, JSON.stringify(report));
});

test('el ruido se conserva y solo la falta de localización consume el margen', { timeout: 120000 }, () => {
  const { report } = evaluateScenario(SCENARIOS.partition, 24000, { odometry: {}, leash: { presupuestoS: 25, maxDistanciaM: 50 }, seed: 7 });
  assert.equal(report.estado, 'completado', JSON.stringify(report));
  assert.equal(report.fronteras_pendientes, 0);
  assert.equal(report.retorno, 'no');
  assert.ok(report.tiempo_exploracion_s > 25);
  assert.ok(report.max_tiempo_sin_localizacion_s < 25);
  assert.ok(report.error_odom_m > 0);
  assert.equal(report.colisiones, 0);
  assert.equal(report.pasos_con_huella_fuera_del_espacio_libre_conocido, 0);
});

function unreferencedRun() {
  const map = createMap(); map.fill(FREE);
  const explorer = createExplorer();
  explorer.state = 'navegando'; explorer.route = [cell(0, -2, map)];
  setLeash(explorer, { presupuestoS: .5, maxDistanciaM: 50 });
  let pose = { x: 0, z: 0, yaw: 0 };
  for (let i = 0; i < 200 && explorer.state !== 'relocalizando'; i++) {
    pose = predictMotion(pose, exploreStep(explorer, map, pose, 1 / 60), 1 / 60);
  }
  return { map, explorer, pose };
}

test('al vencer el margen corta el avance, reobserva y queda detenido si no confirma', () => {
  const { map, explorer, pose: initial } = unreferencedRun();
  assert.equal(explorer.state, 'relocalizando');
  assert.ok(explorer.localizacion.lostSeconds >= .5 - 1e-8);
  let pose = initial;
  for (let i = 0; i < 300 && explorer.state !== 'localizacion_perdida'; i++) {
    const command = exploreStep(explorer, map, pose, 1 / 60);
    assert.equal(command.forward, false); assert.equal(command.back, false);
    pose = predictMotion(pose, command, 1 / 60);
  }
  assert.equal(explorer.state, 'localizacion_perdida');
  assert.equal(explorer.retorno, 'sin_localizacion');
  assert.equal(pose.x, initial.x); assert.equal(pose.z, initial.z);
  const stoppedTime = explorer.localizacion.elapsed;
  assert.deepEqual(exploreStep(explorer, map, pose, 1), { forward: false, back: false, left: false, right: false });
  assert.equal(explorer.localizacion.elapsed, stoppedTime);
});

test('tres confirmaciones recuperan la navegación después de reobservar', () => {
  const { map, explorer, pose } = unreferencedRun();
  for (let i = 0; i < 3; i++) {
    if (i) tickLocalization(explorer.localizacion, .1);
    observeLocalization(explorer.localizacion, { reliable: true, reason: 'esquina_conocida' }, pose);
  }
  assert.equal(explorer.localizacion.lostSeconds, 0);
  assert.equal(explorer.localizacion.confirmed, true);
  exploreStep(explorer, map, pose, .01);
  assert.equal(explorer.state, 'planificando');
});

test('la pausa del operador congela el contador y reanudar no lo borra', () => {
  const { map, explorer, pose } = unreferencedRun();
  pauseExplorer(explorer);
  const before = explorer.localizacion.lostSeconds;
  for (let i = 0; i < 10; i++) exploreStep(explorer, map, pose, 1);
  assert.equal(explorer.localizacion.lostSeconds, before);
  startExplorer(explorer); exploreStep(explorer, map, pose, .01);
  assert.equal(explorer.state, 'relocalizando');
  assert.ok(explorer.localizacion.lostSeconds > before);
});
