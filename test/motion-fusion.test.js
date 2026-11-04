import test from 'node:test';
import assert from 'node:assert/strict';
import { createOdometry, readMotion, SENSORES_RUIDOSOS } from '../src/odometry.js';
import { createMotionFusion, integrateFusedMotion, gateMotionCommand, observeLidarMotion, lidarAdvance } from '../src/motion-fusion.js';
import { sense, SENSOR } from '../src/sensor.js';
import { advance, STEP } from '../src/physics.js';
import { mulberry32 } from '../src/noise.js';
import { createMap, seedFootprint } from '../src/mapping.js';
import { createMissionMapping, updateMissionMapping } from '../src/mission-mapping.js';
import { createExplorer } from '../src/explorer.js';

const origin = { x: 0, z: 0, yaw: 0 }, wall = [[-3, -1, 3, -1]];
test('contacto conserva patinaje medido pero no lo integra como movimiento del cuerpo', () => {
  const odom = createOdometry(origin, { ...SENSORES_RUIDOSOS, seed: 9 }), fusion = createMotionFusion();
  let raw = 0;
  for (let i = 0; i < 60; i++) {
    const reading = readMotion(odom, origin, origin, { forward: true }, STEP, true);
    raw += reading.avance;
    const accepted = integrateFusedMotion(fusion, odom, reading, STEP);
    assert.equal(accepted.avance, reading.avance); assert.equal(accepted.avanceAceptado, 0);
  }
  assert.ok(raw > .3 && raw < .5); assert.equal(odom.x, 0); assert.equal(odom.z, 0);
  assert.equal(odom.config.patinaje, .5); assert.equal(odom.config.biasRueda, 1.01);
  assert.ok(Math.abs(odom.yaw) > 0, 'Conserva giro medido y deriva');
  assert.equal(fusion.suppressedM, raw);
  assert.equal(fusion.events[0].type, 'contacto'); assert.ok(fusion.events[0].encoderM > 0); assert.equal(fusion.events[0].acceptedM, 0);
});
test('el motor deja de empujar y permite soltar, girar y retroceder', () => {
  const fusion = createMotionFusion(), odom = createOdometry(origin, { seed: 7 });
  integrateFusedMotion(fusion, odom, { avance: .006, giro: .02, contact: true, drive: 1 }, STEP);
  assert.equal(gateMotionCommand(fusion, { forward: true }).forward, false);
  const turn = gateMotionCommand(fusion, { forward: true, left: true });
  assert.equal(turn.forward, false); assert.equal(turn.left, true);
  assert.ok(Math.abs(odom.yaw - .02) < 1e-12);
  assert.equal(gateMotionCommand(fusion, { back: true }).back, true);
  assert.equal(fusion.blockedDirection, 0);
  integrateFusedMotion(fusion, odom, { avance: -.006, giro: 0, contact: true, drive: -1 }, STEP);
  assert.equal(gateMotionCommand(fusion, { back: true }).back, false);
  assert.equal(gateMotionCommand(fusion, { forward: true }).forward, true);
});
test('el LiDAR mide acercamiento y alejamiento sin consultar posiciones reales', () => {
  const a = sense(origin, wall).map(r => ({ ...r, angle: 999 }));
  const forward = lidarAdvance(a, sense({ ...origin, z: -.12 }, wall));
  const back = lidarAdvance(a, sense({ ...origin, z: .12 }, wall));
  assert.equal(forward.reliable, true); assert.ok(Math.abs(forward.avance - .12) < .003);
  assert.equal(back.reliable, true); assert.ok(Math.abs(back.avance + .12) < .003);
  const rotated = lidarAdvance(a, sense({ ...origin, yaw: .02 }, wall), SENSOR.fov, .02);
  assert.equal(rotated.reliable, true); assert.ok(Math.abs(rotated.avance) < .005);
});
test('sin bumper, lecturas de pared repetidas detectan el avance fantasma de las ruedas', () => {
  const fusion = createMotionFusion(), odom = createOdometry(origin, { seed: 7 });
  const sensor = { ...SENSOR, noise: .02, random: mulberry32(7) };
  observeLidarMotion(fusion, sense(origin, wall, sensor), odom, sensor);
  let corrected = false, sawSuspect = false;
  for (let i = 1; i <= 120; i++) {
    const applied = gateMotionCommand(fusion, { forward: true });
    integrateFusedMotion(fusion, odom, { avance: applied.forward ? .006 : 0, giro: 0, contact: false, drive: applied.forward ? 1 : 0 }, STEP);
    if (i % 6 === 0) {
      const result = observeLidarMotion(fusion, sense(origin, wall, sensor), odom, sensor);
      sawSuspect ||= result.suspect; corrected ||= result.corrected;
      Object.assign(odom, result.pose);
    }
  }
  assert.equal(sawSuspect, true); assert.equal(corrected, true);
  assert.equal(fusion.stalls, 1); assert.equal(fusion.contacts, 0);
  assert.ok(Math.hypot(odom.x, odom.z) < .025, JSON.stringify(odom));
  assert.ok(fusion.correctedM > .04); assert.equal(fusion.blockedDirection, 1);
});
test('movimiento verdadero hacia la pared no se confunde con patinaje', () => {
  const fusion = createMotionFusion(), odom = createOdometry(origin, { seed: 7 });
  const sensor = { ...SENSOR, noise: .02, random: mulberry32(42) };
  observeLidarMotion(fusion, sense(origin, wall, sensor), odom, sensor);
  for (let i = 1; i <= 30; i++) {
    integrateFusedMotion(fusion, odom, { avance: STEP * 1.01, giro: 0, contact: false, drive: 1 }, STEP);
    if (i % 6 === 0) {
      const reading = observeLidarMotion(fusion, sense({ ...origin, z: -i * STEP }, wall, sensor), odom, sensor);
      assert.equal(reading.corrected, false); assert.equal(reading.suspect, false);
    }
  }
  assert.equal(fusion.stalls, 0); assert.ok(Math.abs(odom.z + .505) < 1e-9);
});
test('no confirma patinaje repitiendo una lectura en el mismo instante ni con datos antiguos', () => {
  const fusion = createMotionFusion(), odom = createOdometry(origin, { seed: 1 }), scan = sense(origin, wall);
  observeLidarMotion(fusion, scan, odom);
  for (let i = 0; i < 12; i++) integrateFusedMotion(fusion, odom, { avance: .006, giro: 0, drive: 1, contact: false }, STEP);
  assert.equal(observeLidarMotion(fusion, scan, odom).suspect, true);
  for (let i = 0; i < 5; i++) assert.equal(observeLidarMotion(fusion, scan, odom).corrected, false);
  assert.equal(fusion.confirmations, 1);
  integrateFusedMotion(fusion, odom, { avance: .2, giro: 0, drive: 1, contact: false }, .8);
  assert.equal(observeLidarMotion(fusion, scan, odom).corrected, false);
  assert.equal(fusion.lidar.reason, 'referencia_antigua');
});
test('giro, pared lateral, suelo y lecturas inválidas no justifican inmovilidad', () => {
  const a = sense(origin, wall);
  assert.equal(lidarAdvance(a, a, SENSOR.fov, .1).reliable, false);
  const side = sense(origin, [[1, -5, 1, 5]]);
  assert.equal(lidarAdvance(side, side).reliable, false);
  const floors = a.map(r => ({ ...r, kind: 'floor' })), invalid = a.map(r => ({ ...r, kind: 'invalid', distance: null }));
  assert.equal(lidarAdvance(floors, floors).reliable, false);
  assert.equal(lidarAdvance(invalid, invalid).reliable, false);
});
test('el choque físico y su mapa quedan acotados sin teletransportar la estimación', () => {
  const start = { x: 0, z: -.54, yaw: 0 }, wallNear = [[-3, -.8, 3, -.8]];
  let truth = { ...start };
  const odom = createOdometry(start, { ...SENSORES_RUIDOSOS, seed: 9 }), map = createMap(); seedFootprint(map, start);
  const mission = createMissionMapping(map, { loopClosure: false }), e = createExplorer();
  let bumps = 0;
  for (let i = 0; i < 120; i++) {
    if (i % 6 === 0) {
      const r = updateMissionMapping(mission, map, sense(truth, wallNear), odom, e, i * STEP, SENSOR, truth.collision);
      Object.assign(odom, r.pose);
    }
    const cmd = gateMotionCommand(mission.motion, { forward: true }), prev = truth;
    truth = advance(truth, cmd, STEP, wallNear); bumps += Number(truth.collision);
    integrateFusedMotion(mission.motion, odom, readMotion(odom, truth, prev, cmd, STEP, truth.collision), STEP);
  }
  assert.equal(bumps, 1);
  assert.ok(Math.abs(truth.z + .55) < 1e-8);
  assert.ok(Math.hypot(odom.x - start.x, odom.z - start.z) < .02);
  assert.ok(Math.hypot(odom.x - truth.x, odom.z - truth.z) < .025);
});
test('una discrepancia sin confirmar no pinta el mapa ni confirma localización', () => {
  const map = createMap(), mission = createMissionMapping(map, { loopClosure: false }), e = createExplorer();
  const odom = createOdometry(origin, { seed: 1 }), scan = sense(origin, wall);
  updateMissionMapping(mission, map, scan, odom, e, 0);
  for (let i = 0; i < 12; i++) integrateFusedMotion(mission.motion, odom, { avance: .006, giro: 0, drive: 1, contact: false }, STEP);
  const before = map.slice(), revision = map.revision, lastConfirmedAt = e.localizacion.lastConfirmedAt;
  const result = updateMissionMapping(mission, map, scan, odom, e, .2);
  assert.equal(result.deferred, true); assert.deepEqual(map.slice(), before); assert.equal(map.revision, revision);
  assert.equal(e.localizacion.lastConfirmedAt, lastConfirmedAt);
});
