import { cell, gridOf, OCCUPIED, FREE } from './mapping.js';
import { SENSOR } from './sensor.js';

// Correlative scan matching: tries small pose offsets around the odometry
// estimate and keeps the one whose wall endpoints best agree with the map.
// Beam bearings come from the known beam layout (hardware knowledge), never
// from ground truth. A motion prior penalizes far jumps, a minimum occupied
// fraction rejects matches against thin evidence, and an already good fit
// skips the search entirely. Local window only: it fixes drift, it cannot
// kidnap. Endpoint projection mirrors the mapper's hit cell, including its
// +.005 m outward nudge, so scores peak at the true pose.
export const SCANMATCH = Object.freeze({
  ventanaM: .15, pasoM: .025,
  ventanaYaw: .04, pasoYaw: .02,
  minImpactos: 8,
  minFraccion: .35,
  buenaFraccion: .95,
  pasoPenal: .5,
  margen: 1,
  alpha: .15,
});

export function alignScan(map, rays, pose, sensor = SENSOR, opts = {}) {
  const cfg = { ...SCANMATCH, ...opts };
  const n = rays.length;
  const ends = [];
  rays.forEach((ray, i) => {
    if (ray.kind !== 'wall' || !Number.isFinite(ray.distance) || ray.distance <= 0) return;
    ends.push({ dist: ray.distance, rel: (n === 1 ? 0 : i / (n - 1) - .5) * sensor.fov });
  });
  const miss = (score, base) => ({ pose: { x: pose.x, z: pose.z, yaw: pose.yaw }, corregido: false, score, base });
  if (ends.length < cfg.minImpactos) return miss(0, 0);
  const g = gridOf(map);
  const creditAt = (x, z, yaw) => {
    let s = 0;
    for (const e of ends) {
      const a = yaw + e.rel, d = e.dist + .005;
      const idx = cell(x - Math.sin(a) * d, z - Math.cos(a) * d, map);
      if (idx < 0) continue;
      if (map[idx] === OCCUPIED) { s += 1; continue; }
      const cx = idx % g.width, cz = Math.floor(idx / g.width);
      let near = false;
      for (let dz = -1; dz <= 1 && !near; dz++) for (let dx = -1; dx <= 1; dx++) {
        const nx2 = cx + dx, nz2 = cz + dz;
        if (nx2 < 0 || nz2 < 0 || nx2 >= g.width || nz2 >= g.height) continue;
        if (map[nz2 * g.width + nx2] === OCCUPIED) { near = true; break; }
      }
      if (near) s += .25;
    }
    return s;
  };
  const base = creditAt(pose.x, pose.z, pose.yaw);
  if (base / ends.length >= cfg.buenaFraccion) return miss(base, base);
  const nx = Math.round(cfg.ventanaM / cfg.pasoM), ny = Math.round(cfg.ventanaYaw / cfg.pasoYaw);
  let best = null, bestVal = base + cfg.margen;
  for (let ix = -nx; ix <= nx; ix++) for (let iz = -nx; iz <= nx; iz++) for (let iy = -ny; iy <= ny; iy++) {
    if (ix === 0 && iz === 0 && iy === 0) continue;
    const credit = creditAt(pose.x + ix * cfg.pasoM, pose.z + iz * cfg.pasoM, pose.yaw + iy * cfg.pasoYaw);
    const val = credit - cfg.pasoPenal * (Math.abs(ix) + Math.abs(iz) + Math.abs(iy));
    if (val > bestVal) { bestVal = val; best = { x: pose.x + ix * cfg.pasoM, z: pose.z + iz * cfg.pasoM, yaw: pose.yaw + iy * cfg.pasoYaw, credit }; }
  }
  if (!best || best.credit / ends.length < cfg.minFraccion) return miss(best?.credit ?? base, base);
  const yaw = Math.atan2(Math.sin(best.yaw), Math.cos(best.yaw));
  return { pose: { x: best.x, z: best.z, yaw }, corregido: true, score: best.credit, base };
}

// Global relocalization against a saved map: coarse correlative search over
// free cells and every heading, a medium sweep, then the local matcher as
// polish. Endpoints landing on known-free cells count against a pose, which
// is what separates the true place from a smeared look-alike. One-shot and
// expensive by design; it runs at startup, not per scan.
export function localizeGlobal(map, rays, sensor = SENSOR, opts = {}) {
  const cfg = { pasoM: .25, pasoYaw: .12, minFraccion: .55, yawCentro: null, yawTol: .45, ...opts };
  const n = rays.length;
  const ends = [];
  rays.forEach((ray, i) => {
    if (ray.kind !== 'wall' || !Number.isFinite(ray.distance) || ray.distance <= 0) return;
    ends.push({ dist: ray.distance, rel: (n === 1 ? 0 : i / (n - 1) - .5) * sensor.fov });
  });
  const miss = (score = 0) => ({ found: false, pose: null, score });
  if (ends.length < 8) return miss();
  const sample = ends.filter((_, i) => i % Math.ceil(ends.length / 60) === 0);
  const g = gridOf(map);
  // Distance-to-wall field (chamfer): credit decays smoothly with endpoint
  // error, so the coarse landscape has a wide basin around the true pose
  // instead of binary hits that only lattice-aligned look-alikes win.
  const field = new Float32Array(map.length).fill(1e9);
  for (let i = 0; i < map.length; i++) if (map[i] === OCCUPIED) field[i] = 0;
  const res = g.resolution, diag = res * Math.SQRT2;
  for (let z = 0; z < g.height; z++) for (let x = 0; x < g.width; x++) {
    const i = z * g.width + x;
    if (x > 0) field[i] = Math.min(field[i], field[i - 1] + res);
    if (z > 0) field[i] = Math.min(field[i], field[i - g.width] + res);
    if (x > 0 && z > 0) field[i] = Math.min(field[i], field[i - g.width - 1] + diag);
    if (x < g.width - 1 && z > 0) field[i] = Math.min(field[i], field[i - g.width + 1] + diag);
  }
  for (let z = g.height - 1; z >= 0; z--) for (let x = g.width - 1; x >= 0; x--) {
    const i = z * g.width + x;
    if (x < g.width - 1) field[i] = Math.min(field[i], field[i + 1] + res);
    if (z < g.height - 1) field[i] = Math.min(field[i], field[i + g.width] + res);
    if (x < g.width - 1 && z < g.height - 1) field[i] = Math.min(field[i], field[i + g.width + 1] + diag);
    if (x > 0 && z < g.height - 1) field[i] = Math.min(field[i], field[i + g.width - 1] + diag);
  }
  const creditAt = (x, z, yaw) => {
    let s = 0;
    for (const e of sample) {
      const a = yaw + e.rel, idx = cell(x - Math.sin(a) * (e.dist + .005), z - Math.cos(a) * (e.dist + .005), map);
      if (idx >= 0) s += Math.max(0, 1 - field[idx] / .35);
    }
    return s / sample.length;
  };
  const wrapA = a => Math.atan2(Math.sin(a), Math.cos(a));
  const stride = Math.max(1, Math.round(cfg.pasoM / g.resolution));
  const yaws = Math.max(8, Math.round(Math.PI * 2 / cfg.pasoYaw));
  const coarse = [];
  for (let row = 0; row < g.height; row += stride) for (let col = 0; col < g.width; col += stride) {
    if (map[row * g.width + col] !== FREE) continue;
    const px = g.originX + (col + .5) * g.resolution, pz = g.originZ + (row + .5) * g.resolution;
    for (let k = 0; k < yaws; k++) {
      const yaw = k / yaws * Math.PI * 2;
      // Compass prior: headings far from the measured absolute heading
      // are not searched at all — cheaper and removes mirrored rivals.
      if (cfg.yawCentro != null && Math.abs(wrapA(yaw - cfg.yawCentro)) > cfg.yawTol) continue;
      const score = creditAt(px, pz, yaw);
      if (score >= .25) coarse.push({ x: px, z: pz, yaw, score });
    }
  }
  if (!coarse.length) return miss();
  coarse.sort((a, b) => b.score - a.score);
  const tops = [];
  for (const c of coarse) {
    if (tops.length >= 16) break;
    if (tops.every(p => Math.hypot(p.x - c.x, p.z - c.z) > .4 || Math.abs(wrapA(p.yaw - c.yaw)) > .3)) tops.push(c);
  }
  // Refine every finalist the same way; only then are scores comparable.
  const candidatos = tops.map(c => {
    let refined = c;
    for (let dx = -.25; dx <= .25; dx += .05) for (let dz = -.25; dz <= .25; dz += .05) for (let dy = -.2; dy <= .2; dy += .025) {
      const score = creditAt(c.x + dx, c.z + dz, c.yaw + dy);
      if (score > refined.score) refined = { x: c.x + dx, z: c.z + dz, yaw: c.yaw + dy, score };
    }
    return refined;
  }).sort((a, b) => b.score - a.score);
  const best = candidatos[0];
  const fix = alignScan(map, rays, best, sensor);
  const pose = fix.corregido ? fix.pose : { x: best.x, z: best.z, yaw: wrapA(best.yaw) };
  const score = Math.max(best.score, fix.score / ends.length);
  // A distinct rival explaining the view almost as well is perceptual
  // aliasing: report both places instead of guessing one.
  const rival = candidatos[1];
  const ambiguo = Boolean(rival && rival.score >= score - .03);
  const found = score >= cfg.minFraccion && !ambiguo;
  return { found, pose: found ? pose : null, score, ambiguo, candidatos };
}

// Fraction of wall endpoints agreeing with the map at one fixed pose
// (1 = every endpoint on a known wall). Zero search window: the local
// matcher evaluates the given pose and reports its base credit.
export function scorePose(map, rays, pose, sensor = SENSOR) {
  const n = rays.length;
  let ends = 0;
  for (const ray of rays) if (ray.kind === 'wall' && Number.isFinite(ray.distance) && ray.distance > 0) ends++;
  if (!ends) return 0;
  return alignScan(map, rays, pose, sensor, { ventanaM: 0, ventanaYaw: 0 }).base / ends;
}

// Complementary filter: applies a fraction of the match correction. Real
// drift persists across scans so it still gets tracked; an isolated
// mismatch barely nudges the belief instead of scattering the map.
export function blendPose(bp, target, alpha = .3) {
  const dyaw = Math.atan2(Math.sin(target.yaw - bp.yaw), Math.cos(target.yaw - bp.yaw));
  const yaw = bp.yaw + dyaw * alpha;
  return { x: bp.x + (target.x - bp.x) * alpha, z: bp.z + (target.z - bp.z) * alpha, yaw: Math.atan2(Math.sin(yaw), Math.cos(yaw)) };
}
