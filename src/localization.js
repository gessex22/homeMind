import { cell, gridOf, UNKNOWN, OCCUPIED } from './mapping.js';
import { SENSOR } from './sensor.js';
import { SPEED, TURN_SPEED } from './motion.js';

export const LOCALIZATION = Object.freeze({ freshness: .6, confirmations: 3, confirmationSpan: .18, maxEvidenceGap: .3 });

// Compare with the map BEFORE inserting this scan. Sensor beam layout is
// known; true simulator bearings and true pose never enter this assessment.
export function assessLocalization(map, rays, pose, sensor = SENSOR) {
  const g = gridOf(map), ends = [];
  rays.forEach((ray, i) => {
    if (ray.kind === 'wall' && Number.isFinite(ray.distance) && ray.distance > 0) {
      ends.push({ distance: ray.distance + .005, angle: pose.yaw + (rays.length === 1 ? 0 : i / (rays.length - 1) - .5) * sensor.fov });
    }
  });
  const reject = (reason, overlap = 0, ambiguity = 1) => ({ reliable: false, reason, overlap, ambiguity, hits: ends.length });
  if (ends.length < 8) return reject('sin_referencias_suficientes');
  function score(dx, dz, alternative = false) {
    let sum = 0;
    for (const end of ends) {
      const i = cell(pose.x + dx - Math.sin(end.angle) * end.distance, pose.z + dz - Math.cos(end.angle) * end.distance, map);
      if (alternative) {
        let obstructed = false;
        for (let d = Math.max(0, end.distance - .6); d < end.distance - .15; d += g.resolution / 2) {
          const before = cell(pose.x + dx - Math.sin(end.angle) * d, pose.z + dz - Math.cos(end.angle) * d, map);
          if (before >= 0 && map[before] === OCCUPIED) { obstructed = true; break; }
        }
        if (obstructed) continue;
      }
      // Unknown space cannot disprove another pose. Only known contradictory
      // free/occupied observations make a competing pose less plausible.
      if (i < 0 || map[i] === UNKNOWN) { if (alternative) sum++; continue; }
      if (map[i] === OCCUPIED) { sum++; continue; }
      const x = i % g.width, z = Math.floor(i / g.width);
      let near = false;
      for (let row = -1; row <= 1 && !near; row++) for (let col = -1; col <= 1; col++) {
        const nx = x + col, nz = z + row;
        if (nx >= 0 && nz >= 0 && nx < g.width && nz < g.height && map[nz * g.width + nx] === OCCUPIED) { near = true; break; }
      }
      if (near) sum += .25;
    }
    return sum / ends.length;
  }
  const overlap = score(0, 0);
  if (overlap < .7) return reject('lectura_no_coincide_con_mapa', overlap);
  let alternative = 0;
  for (let i = 0; i < 8; i++) {
    const angle = i * Math.PI / 4;
    alternative = Math.max(alternative, score(.2 * Math.cos(angle), .2 * Math.sin(angle), true));
  }
  // Along a uniform wall/corridor, another translation fits equally well.
  // Receiving a good range reading there does not confirm a unique position.
  if (overlap - alternative < .12) return reject('geometria_ambigua', overlap, alternative);
  return { reliable: true, reason: 'ajuste_distinguible', overlap, ambiguity: alternative, hits: ends.length };
}

export function createLocalizationMonitor(options = {}) {
  const { initialPose = { x: 0, z: 0, yaw: 0 }, ...overrides } = options;
  const config = { ...LOCALIZATION, ...overrides };
  if (!Number.isFinite(config.freshness) || config.freshness < 0 || !Number.isInteger(config.confirmations) || config.confirmations < 2
    || !Number.isFinite(config.confirmationSpan) || config.confirmationSpan <= 0
    || !Number.isFinite(config.maxEvidenceGap) || config.maxEvidenceGap < config.confirmationSpan) throw new RangeError('Parámetros de localización inválidos');
  return {
    config, elapsed: 0, lostSeconds: 0, maxLostSeconds: 0,
    confirmed: Boolean(initialPose), status: initialPose ? 'confirmada' : 'sin_confirmar',
    lastConfirmedAt: initialPose ? 0 : null, lastReliablePose: initialPose ? { ...initialPose } : null,
    streak: 0, firstEvidenceAt: null, pendingAt: null, pendingPose: null, lastEvidence: null, invalidatedAt: null,
  };
}

function refresh(monitor) {
  const age = monitor.lastConfirmedAt === null ? Infinity : monitor.elapsed - monitor.lastConfirmedAt;
  monitor.confirmed = monitor.invalidatedAt === null && age <= monitor.config.freshness + 1e-9;
  monitor.lostSeconds = monitor.invalidatedAt !== null ? monitor.elapsed - monitor.invalidatedAt : monitor.confirmed ? 0 : monitor.lastConfirmedAt === null ? monitor.elapsed : Math.max(0, age - monitor.config.freshness);
  monitor.maxLostSeconds = Math.max(monitor.maxLostSeconds, monitor.lostSeconds);
  monitor.status = monitor.confirmed ? 'confirmada' : monitor.streak > 0 ? 'reconfirmando' : 'sin_confirmar';
}
// A graph correction changes the pose frame. Old confirmation cannot authorize
// translation; preserve an already-running loss timer and request new evidence.
export function invalidateLocalization(monitor) {
  if (monitor.confirmed) monitor.invalidatedAt = monitor.elapsed;
  monitor.lastReliablePose = null;
  monitor.streak = 0; monitor.firstEvidenceAt = null; monitor.pendingAt = null; monitor.pendingPose = null;
  refresh(monitor);
}

export function tickLocalization(monitor, dt) {
  if (!Number.isFinite(dt) || dt < 0) throw new RangeError('El tiempo de localización debe ser positivo');
  monitor.elapsed += dt;
  refresh(monitor);
}

export function observeLocalization(monitor, evidence, pose) {
  monitor.lastEvidence = { ...evidence };
  if (!evidence.reliable || ![pose.x, pose.z, pose.yaw].every(Number.isFinite)) {
    monitor.streak = 0; monitor.firstEvidenceAt = null; monitor.pendingAt = null; monitor.pendingPose = null;
    refresh(monitor); return;
  }
  const gap = monitor.pendingAt === null ? Infinity : monitor.elapsed - monitor.pendingAt;
  if (gap < .05) return; // Re-reading the same instant is not redundant evidence.
  const distance = monitor.pendingPose ? Math.hypot(pose.x - monitor.pendingPose.x, pose.z - monitor.pendingPose.z) : Infinity;
  const turn = monitor.pendingPose ? Math.abs(Math.atan2(Math.sin(pose.yaw - monitor.pendingPose.yaw), Math.cos(pose.yaw - monitor.pendingPose.yaw))) : Infinity;
  const consistent = gap <= monitor.config.maxEvidenceGap + 1e-9 && distance <= SPEED * gap + .08 && turn <= TURN_SPEED * gap + .08;
  monitor.streak = consistent ? monitor.streak + 1 : 1;
  if (!consistent) monitor.firstEvidenceAt = monitor.elapsed;
  monitor.pendingAt = monitor.elapsed; monitor.pendingPose = { x: pose.x, z: pose.z, yaw: pose.yaw };
  if (monitor.streak >= monitor.config.confirmations && monitor.elapsed - monitor.firstEvidenceAt >= monitor.config.confirmationSpan - 1e-9) {
    monitor.lastConfirmedAt = monitor.elapsed; monitor.lastReliablePose = { ...monitor.pendingPose };
    monitor.invalidatedAt = null;
  }
  refresh(monitor);
}
