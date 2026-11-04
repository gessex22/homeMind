// Robot-side fusion: encoder/gyro measurements, bumper and calibrated LiDAR.
// No geometry, physical pose, simulator collision solver or object inventory.
import { driveValue } from './motion.js';
import { integrateMeasurement } from './odometry.js';
import { SENSOR } from './sensor.js';

export const MOTION_FUSION = Object.freeze({ minWindow: .15, maxWindow: .5, minEncoderTravel: .045,
  maxTurn: .035, stableDistance: .012, minForwardNormal: .7, confirmations: 2 });
const median = values => { const a = [...values].sort((x, y) => x - y); return a.length ? (a[Math.floor((a.length - 1) / 2)] + a[Math.ceil((a.length - 1) / 2)]) / 2 : NaN; };
export function createMotionFusion({ enabled = true } = {}) {
  return { enabled, config: { ...MOTION_FUSION }, time: 0, reference: null, wheelWindow: 0, acceptedWindow: 0, gyroWindow: 0,
    blockedDirection: 0, lastDrive: 0, confirmations: 0, pendingStall: false,
    status: enabled ? 'sin_referencia' : 'desactivada', contacts: 0, stalls: 0, suppressedM: 0, correctedM: 0,
    lastRaw: { avance: 0, giro: 0 }, lastAccepted: 0, lidar: null, lastScanAt: -Infinity, events: [] };
}
export function integrateFusedMotion(fusion, odom, measurement, dt) {
  fusion.time += dt; fusion.lastRaw = { ...measurement };
  let accepted = measurement.avance;
  if (fusion.enabled && measurement.contact && measurement.drive !== 0) {
    if (!fusion.blockedDirection) {
      fusion.contacts++;
      fusion.events.push({ time: fusion.time, type: 'contacto', encoderM: measurement.avance, acceptedM: 0, giro: measurement.giro, direction: Math.sign(measurement.drive) });
      if (fusion.events.length > 100) fusion.events.shift();
    }
    fusion.blockedDirection = Math.sign(measurement.drive); fusion.status = 'contacto';
  }
  if (fusion.enabled && fusion.blockedDirection && Math.sign(measurement.avance) === fusion.blockedDirection) accepted = 0;
  fusion.suppressedM += Math.abs(measurement.avance - accepted);
  fusion.wheelWindow += measurement.avance; fusion.acceptedWindow += accepted; fusion.gyroWindow += measurement.giro;
  fusion.lastAccepted = accepted;
  integrateMeasurement(odom, { avance: accepted, giro: measurement.giro });
  return { ...measurement, avanceAceptado: accepted };
}
// Latch the stop until the operator releases that direction or requests escape.
// Rotation and retreat remain available; the autonomous bumper reflex is kept.
export function gateMotionCommand(fusion, requested) {
  const drive = driveValue(requested);
  fusion.lastDrive = drive;
  if (!fusion.enabled || !fusion.blockedDirection) return { ...requested };
  if (Math.sign(drive) !== fusion.blockedDirection) {
    fusion.blockedDirection = 0; fusion.confirmations = 0; fusion.status = 'reobservando';
    return { ...requested };
  }
  return { ...requested, forward: false, back: false };
}
function points(rays, fov, turn = 0) {
  const result = [];
  for (let i = 0; i < rays.length; i++) {
    const a = (rays.length === 1 ? 0 : i / (rays.length - 1) - .5) * fov;
    if (Math.abs(a) > 25 * Math.PI / 180 || rays[i].kind !== 'wall' || !Number.isFinite(rays[i].distance) || rays[i].distance <= .05) continue;
    result.push({ x: -Math.sin(a + turn) * rays[i].distance, z: -Math.cos(a + turn) * rays[i].distance });
  }
  return result;
}
function plane(samples) {
  if (samples.length < 20) return null;
  const x = samples.reduce((v, p) => v + p.x, 0) / samples.length, z = samples.reduce((v, p) => v + p.z, 0) / samples.length;
  let xx = 0, zz = 0, xz = 0;
  for (const p of samples) { xx += (p.x - x) ** 2; zz += (p.z - z) ** 2; xz += (p.x - x) * (p.z - z); }
  const theta = .5 * Math.atan2(2 * xz, xx - zz), nx = -Math.sin(theta), nz = Math.cos(theta);
  const residuals = samples.map(p => nx * (p.x - x) + nz * (p.z - z));
  const rms = Math.sqrt(residuals.reduce((v, r) => v + r * r, 0) / samples.length);
  if (rms > .035 || (xx + zz) / samples.length < .0005 || Math.abs(nz) < .7) return null;
  return { nx, nz, offset: nx * x + nz * z };
}
// Estimate ONLY forward/back displacement observable against a static plane.
// Sideways motion along a uniform wall and large rotations are unobservable here.
export function lidarAdvance(previous, current, fov = SENSOR.fov, turn = 0) {
  if (Math.abs(turn) > MOTION_FUSION.maxTurn || previous.length !== current.length) return { reliable: false, reason: 'giro_o_calibracion' };
  const wall = plane(points(previous, fov)), fresh = points(current, fov, turn);
  if (!wall || fresh.length < 20) return { reliable: false, reason: 'sin_pared_frontal' };
  const divisor = wall.nx * Math.sin(turn / 2) + wall.nz * Math.cos(turn / 2);
  if (Math.abs(divisor) < MOTION_FUSION.minForwardNormal) return { reliable: false, reason: 'movimiento_no_observable' };
  const residuals = fresh.map(p => wall.nx * p.x + wall.nz * p.z - wall.offset), value = median(residuals);
  const inliers = residuals.filter(r => Math.abs(r - value) < .05);
  if (inliers.length < fresh.length * .85 || Math.abs(value) > .3) return { reliable: false, reason: 'geometria_cambio' };
  const spread = median(residuals.map(r => Math.abs(r - value)));
  if (spread > .025) return { reliable: false, reason: 'lectura_inconsistente' };
  return { reliable: true, avance: median(inliers) / divisor, spread, hits: inliers.length };
}
function setReference(fusion, rays, pose, fov) {
  fusion.reference = { rays: rays.map(r => ({ kind: r.kind, distance: r.distance })), yaw: pose.yaw, fov, time: fusion.time };
  fusion.wheelWindow = 0; fusion.acceptedWindow = 0; fusion.gyroWindow = 0;
}
export function observeLidarMotion(fusion, rays, pose, sensor = SENSOR) {
  if (!fusion.enabled) return { pose, corrected: false, suspect: false };
  if (fusion.time - fusion.lastScanAt < .05) return { pose, corrected: false, suspect: false };
  fusion.lastScanAt = fusion.time;
  if (!fusion.reference) { setReference(fusion, rays, pose, sensor.fov); return { pose, corrected: false, suspect: false }; }
  if (fusion.reference.fov !== sensor.fov) { fusion.confirmations = 0; setReference(fusion, rays, pose, sensor.fov); return { pose, corrected: false, suspect: false }; }
  const age = fusion.time - fusion.reference.time;
  if (age > fusion.config.maxWindow) {
    fusion.confirmations = 0;
    fusion.lidar = { reliable: false, reason: 'referencia_antigua' };
    if (!fusion.blockedDirection) fusion.status = 'lidar_insuficiente';
    setReference(fusion, rays, pose, sensor.fov);
    return { pose, corrected: false, suspect: false };
  }
  const evidence = lidarAdvance(fusion.reference.rays, rays, sensor.fov, fusion.gyroWindow);
  fusion.lidar = evidence;
  if (!evidence.reliable) {
    fusion.confirmations = 0;
    if (!fusion.blockedDirection) fusion.status = 'lidar_insuficiente';
    setReference(fusion, rays, pose, sensor.fov);
    return { pose, corrected: false, suspect: false };
  }
  if (age < fusion.config.minWindow) return { pose, corrected: false, suspect: false };
  const wheel = fusion.wheelWindow;
  const stalled = Math.abs(wheel) >= fusion.config.minEncoderTravel && Math.abs(evidence.avance) <= fusion.config.stableDistance && Math.abs(evidence.avance) < Math.abs(wheel) * .25;
  if (!stalled) {
    fusion.confirmations = 0;
    if (!fusion.blockedDirection) fusion.status = 'coherente';
    setReference(fusion, rays, pose, sensor.fov);
    return { pose, corrected: false, suspect: false };
  }
  fusion.confirmations++;
  if (!fusion.blockedDirection) fusion.status = 'verificando_patinaje';
  if (fusion.confirmations < fusion.config.confirmations) {
    return { pose, corrected: false, suspect: true };
  }
  const difference = fusion.acceptedWindow - evidence.avance;
  const yaw = fusion.reference.yaw + fusion.gyroWindow / 2;
  const corrected = { ...pose, x: pose.x + Math.sin(yaw) * difference, z: pose.z + Math.cos(yaw) * difference };
  if (!fusion.blockedDirection) { fusion.stalls++; fusion.pendingStall = true; }
  fusion.blockedDirection = Math.sign(wheel); fusion.status = 'patinaje_confirmado';
  fusion.correctedM += Math.abs(difference);
  fusion.events.push({ time: fusion.time, type: 'lidar', encoderM: wheel, lidarM: evidence.avance, correctionM: difference, confirmations: fusion.confirmations });
  if (fusion.events.length > 100) fusion.events.shift();
  setReference(fusion, rays, corrected, sensor.fov); fusion.confirmations = 0;
  return { pose: corrected, corrected: true, suspect: false };
}
export function consumeLidarStall(fusion) { const pending = fusion.pendingStall; fusion.pendingStall = false; return pending; }
export function motionFusionSummary(fusion) {
  return { enabled: fusion.enabled, status: fusion.status, contacts: fusion.contacts, stalls: fusion.stalls,
    encoderM: fusion.lastRaw.avance, acceptedM: fusion.lastAccepted, suppressedM: fusion.suppressedM,
    correctedM: fusion.correctedM, blockedDirection: fusion.blockedDirection, lidar: fusion.lidar, events: fusion.events.map(e => ({ ...e })) };
}
