// Simulated camera detector (host adapter, like sense() for the lidar):
// consumes true object positions only to simulate perception. Returns
// bearing + distance estimates with noise for objects inside the field of
// view, in range, and not occluded by a wall. The engine receives these
// detections and nothing else about the objects.
import { gaussian } from './noise.js';

export const CAMERA = Object.freeze({ range: 4.5, fov: 80 * Math.PI / 180, noiseDist: .05, noiseBearing: .02 });

const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));
function segmentsCross(p, q, [x1, z1, x2, z2], tMax = .98) {
  const rx = q.x - p.x, rz = q.z - p.z, sx = x2 - x1, sz = z2 - z1;
  const den = rx * sz - rz * sx;
  if (Math.abs(den) < 1e-10) return false;
  const t = ((x1 - p.x) * sz - (z1 - p.z) * sx) / den;
  const u = ((x1 - p.x) * rz - (z1 - p.z) * rx) / den;
  return t > .02 && t < tMax && u >= 0 && u <= 1;
}

export function detectObjects(pose, objetos, walls, config = CAMERA) {
  const out = [];
  for (const o of objetos ?? []) {
    const dx = o.x - pose.x, dz = o.z - pose.z;
    const dist = Math.hypot(dx, dz);
    if (dist > config.range || dist < .05) continue;
    // Project convention: forward is (−sin yaw, −cos yaw).
    const bearing = wrap(Math.atan2(-dx, -dz) - pose.yaw);
    if (Math.abs(bearing) > config.fov / 2) continue;
    // A box-shaped object occludes its own centre: ignore crossings in the
    // last stretch, where the object's own near face would be.
    const propio = o.w && o.d ? Math.hypot(o.w, o.d) / 2 : 0;
    const tMax = propio ? Math.max(.05, (dist - propio) / dist) : .98;
    if ((walls ?? []).some(w => segmentsCross(pose, o, w, tMax))) continue;
    const random = config.random;
    out.push({
      label: o.label,
      bearing: random ? bearing + gaussian(random) * (config.noiseBearing ?? 0) : bearing,
      dist: random ? Math.max(.05, dist + gaussian(random) * (config.noiseDist ?? 0)) : dist,
    });
  }
  return out.sort((a, b) => a.dist - b.dist);
}
