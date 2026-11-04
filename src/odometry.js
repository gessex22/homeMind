import { SPEED, driveValue } from './motion.js';
import { mulberry32, gaussian } from './noise.js';

// Measured odometry: wheel encoders report travelled distance off the TRUE
// motion (with scale bias, slip noise, and spin while stuck) and a gyro
// reports yaw deltas with small noise plus an always-on bias drift.
// It never sees ground truth directly; error grows like the real thing.
export const SENSORES_EXACTOS = Object.freeze({ biasRueda: 1, ruidoRueda: 0, patinaje: 0, ruidoGiro: 0, derivaGiro: 0, ruidoBrujula: 0 });
export const SENSORES_RUIDOSOS = Object.freeze({ biasRueda: 1.01, ruidoRueda: .01, patinaje: .5, ruidoGiro: .002, derivaGiro: .0002, ruidoBrujula: .03 });

export function createOdometry(pose, opts = {}) {
  const config = { ...SENSORES_EXACTOS, ...opts };
  if (![config.biasRueda, config.ruidoRueda, config.patinaje, config.ruidoGiro, config.derivaGiro, config.ruidoBrujula].every(Number.isFinite)
    || config.biasRueda <= 0 || config.ruidoRueda < 0 || config.patinaje < 0 || config.patinaje > 1
    || config.ruidoGiro < 0 || config.ruidoBrujula < 0) {
    throw new RangeError('Parámetros de sensores inválidos');
  }
  const seed = Number.isInteger(opts.seed) ? opts.seed >>> 0 : (Math.random() * 2 ** 32) >>> 0;
  // The compass is separate hardware with its own noise stream: its
  // draws must not shift the wheel/gyro noise sequence other tests and
  // missions are calibrated against.
  return { x: pose.x, z: pose.z, yaw: pose.yaw, config: { ...config, seed }, random: mulberry32(seed), brujulaRandom: mulberry32((seed ^ 0xb201) >>> 0) };
}

export function odomPose(odom) {
  return { x: odom.x, z: odom.z, yaw: odom.yaw };
}

// Simulated hardware adapter. Only this reader consumes physical motion.
// The estimator below receives measurements, never simulator poses.
export function readMotion(odom, truth, prevTruth, command, dt, collision = false) {
  const { biasRueda, ruidoRueda, patinaje, ruidoGiro, derivaGiro, ruidoBrujula } = odom.config;
  const drive = driveValue(command);
  const dx = truth.x - prevTruth.x, dz = truth.z - prevTruth.z;
  const signed = dx * -Math.sin(truth.yaw) + dz * -Math.cos(truth.yaw);
  const trueDYaw = Math.atan2(Math.sin(truth.yaw - prevTruth.yaw), Math.cos(truth.yaw - prevTruth.yaw));
  const avance = collision
    ? drive * SPEED * dt * patinaje * (.5 + odom.random() * .5)
    : signed * biasRueda + (signed !== 0 ? gaussian(odom.random) * ruidoRueda * Math.abs(signed) : 0);
  const giro = trueDYaw + gaussian(odom.random) * ruidoGiro + derivaGiro * dt;
  // Compass: absolute heading measurement (noisy, drift-free). It is a
  // sensor reading produced by this host adapter; the engine uses it to
  // prune relocalization headings, never as ground-truth pose.
  const ruidoB = gaussian(odom.brujulaRandom ?? odom.random) * (ruidoBrujula ?? 0);
  const brujula = Math.atan2(Math.sin(truth.yaw + ruidoB), Math.cos(truth.yaw + ruidoB));
  return { avance, giro, contact: Boolean(collision), drive, brujula };
}
export function integrateMeasurement(odom, { avance, giro }) {
  odom.yaw = Math.atan2(Math.sin(odom.yaw + giro), Math.cos(odom.yaw + giro));
  odom.x = odom.x - Math.sin(odom.yaw) * avance;
  odom.z = odom.z - Math.cos(odom.yaw) * avance;
  return { avance, giro };
}
// Raw encoder-only baseline, retained for sensor tests and paired evaluations.
export function updateMedicion(odom, truth, prevTruth, command, dt, collision = false) {
  return integrateMeasurement(odom, readMotion(odom, truth, prevTruth, command, dt, collision));
}
