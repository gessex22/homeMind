import test from 'node:test';
import assert from 'node:assert/strict';
import { createOdometry, updateMedicion, odomPose, SENSORES_RUIDOSOS } from '../src/odometry.js';
import { predictMotion, STEP } from '../src/motion.js';
import { sense, SENSOR } from '../src/sensor.js';
import { mulberry32 } from '../src/noise.js';
import { SCENARIOS } from '../src/world.js';
import { evaluateScenario } from '../scripts/evaluate-room.mjs';

function drive(odom, truth, command, steps, collision = false) {
  for (let i = 0; i < steps; i++) {
    const prev = { ...truth };
    Object.assign(truth, predictMotion(truth, command, STEP));
    updateMedicion(odom, truth, prev, command, STEP, collision);
  }
  return truth;
}

test('sin ruido la medición sigue la verdad exacta', () => {
  const odom = createOdometry({ x: 0, z: 0, yaw: 0 }, { seed: 1 });
  const truth = { x: 0, z: 0, yaw: 0 };
  const commands = [{ forward: true }, { forward: true, left: true }, { left: true }, { back: true, right: true }];
  for (const command of commands) drive(odom, truth, command, 60);
  const p = odomPose(odom);
  assert.ok(Math.abs(p.x - truth.x) < 1e-9 && Math.abs(p.z - truth.z) < 1e-9 && Math.abs(p.yaw - truth.yaw) < 1e-9);
});

test('el reposo no mueve la posición; el giro apenas deriva', () => {
  const exact = createOdometry({ x: 1, z: 2, yaw: .5 }, { seed: 1 });
  const still = { x: 1, z: 2, yaw: .5 };
  drive(exact, still, {}, 600);
  assert.deepEqual(odomPose(exact), { x: 1, z: 2, yaw: .5 });
  const noisy = createOdometry({ x: 1, z: 2, yaw: .5 }, { ...SENSORES_RUIDOSOS, seed: 3 });
  const truth = { x: 1, z: 2, yaw: .5 };
  drive(noisy, truth, {}, 600);
  const p = odomPose(noisy);
  assert.equal(p.x, 1);
  assert.equal(p.z, 2);
  assert.ok(Math.abs(p.yaw - .5) < .15);
});

test('con ruido el error crece con el recorrido', () => {
  const odom = createOdometry({ x: 0, z: 0, yaw: 0 }, { ...SENSORES_RUIDOSOS, seed: 7 });
  const truth = { x: 0, z: 0, yaw: 0 };
  const errors = [];
  for (let leg = 0; leg < 4; leg++) {
    drive(odom, truth, { forward: true }, 300);
    drive(odom, truth, { left: true }, 60);
    errors.push(Math.hypot(odom.x - truth.x, odom.z - truth.z));
  }
  assert.ok(errors[3] > errors[0]);
  assert.ok(errors[3] > .05);
});

test('atascado las ruedas patinan y la creencia avanza sin moverse', () => {
  const odom = createOdometry({ x: 0, z: 0, yaw: 0 }, { ...SENSORES_RUIDOSOS, seed: 9 });
  const truth = { x: 0, z: 0, yaw: 0 };
  for (let i = 0; i < 60; i++) updateMedicion(odom, truth, { ...truth }, { forward: true }, STEP, true);
  assert.ok(Math.hypot(odom.x, odom.z) > .1);
  const quiet = createOdometry({ x: 0, z: 0, yaw: 0 }, { ...SENSORES_RUIDOSOS, seed: 9 });
  for (let i = 0; i < 60; i++) updateMedicion(quiet, truth, { ...truth }, { forward: true }, STEP, false);
  const q = odomPose(quiet);
  assert.equal(q.x, 0);
  assert.equal(q.z, 0);
  assert.ok(Math.abs(q.yaw) < .05);
});

test('parámetros de sensores inválidos se rechazan', () => {
  assert.throws(() => createOdometry({ x: 0, z: 0, yaw: 0 }, { biasRueda: 0 }), RangeError);
  assert.throws(() => createOdometry({ x: 0, z: 0, yaw: 0 }, { patinaje: 2 }), RangeError);
  assert.throws(() => createOdometry({ x: 0, z: 0, yaw: 0 }, { ruidoGiro: -1 }), RangeError);
});

test('el ruido del sensor dispersa sin sesgo sistemático', () => {
  const pose = { x: 0, z: 0, yaw: 0 }, walls = [[-3, -1, 3, -1]];
  const truth = sense(pose, walls, { ...SENSOR, rays: 1 })[0].distance;
  const random = mulberry32(7);
  const samples = Array.from({ length: 200 }, () => sense(pose, walls, { ...SENSOR, rays: 1, noise: .05, random })[0].distance);
  const mean = samples.reduce((sum, v) => sum + v, 0) / samples.length;
  assert.ok(Math.abs(mean - truth) < .02);
  assert.ok(samples.some(v => v !== truth));
});

test('la abertura con ruido se completa con deriva acotada', { timeout: 120000 }, () => {
  const { report } = evaluateScenario(SCENARIOS.partition, 24000, { odometry: {}, seed: 7 });
  assert.equal(report.odometria, 'ruidosa');
  assert.equal(report.estado, 'completado', JSON.stringify(report));
  assert.equal(report.fronteras_pendientes, 0);
  assert.equal(report.pasos_con_huella_fuera_del_espacio_libre_conocido, 0);
  assert.equal(report.colisiones, 0);
  assert.ok(report.error_odom_m > 0 && report.error_odom_m < .25, JSON.stringify(report));
});

test('la casa con ruido se completa sin pasos inseguros', { timeout: 180000 }, () => {
  const { report } = evaluateScenario(SCENARIOS.casa, 24000, { odometry: {}, seed: 7 });
  assert.equal(report.estado, 'completado', JSON.stringify(report));
  assert.equal(report.fronteras_pendientes, 0);
  assert.equal(report.colisiones, 0);
  assert.equal(report.pasos_con_huella_fuera_del_espacio_libre_conocido, 0);
  assert.ok(report.suelo_observado_porcentaje > 95, JSON.stringify(report));
  assert.ok(report.error_odom_m > 0 && report.error_odom_m < .35, JSON.stringify(report));
});

test('la sala grande con ruido expone deriva acumulada e incompletitud', { timeout: 180000 }, () => {
  const { report } = evaluateScenario(SCENARIOS.large, 36000, { odometry: {}, seed: 7 });
  assert.equal(report.estado, 'limitado', JSON.stringify(report));
  assert.equal(report.aprobado, false);
  assert.ok(report.fronteras_pendientes > 0);
  assert.equal(report.colisiones, 0);
  assert.equal(report.pasos_con_huella_fuera_del_espacio_libre_conocido, 0);
  assert.ok(report.suelo_observado_porcentaje > 0);
  assert.ok(report.error_odom_m > 0 && report.error_odom_m < 1.5, 'Deriva real acotada por corrección local');
});
