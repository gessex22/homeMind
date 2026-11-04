import { SCENARIOS } from './world.js';
import { gaussian } from './noise.js';
// Only the sensor consumes ground-truth geometry as perception.
export const SENSOR = { range: 3.8, fov: 80 * Math.PI / 180, rays: 121, ground: true, noise: 0 };
export const ROOM_WALLS = SCENARIOS.empty.walls;
// Perception provides calibrated bearings, never the simulator's global yaw.
export function projectScan(rays, estimatedPose, config = SENSOR) {
  return rays.map((ray, i) => ({ ...ray,
    angle: estimatedPose.yaw + (rays.length === 1 ? 0 : i / (rays.length - 1) - .5) * config.fov,
  }));
}
const cross = (ax, az, bx, bz) => ax * bz - az * bx;
export function sense(pose, walls = ROOM_WALLS, config = SENSOR) {
  return Array.from({ length: config.rays }, (_, i) => {
    const angle = pose.yaw + (config.rays === 1 ? 0 : i / (config.rays - 1) - .5) * config.fov;
    const dx = -Math.sin(angle), dz = -Math.cos(angle);
    let distance = Infinity;
    for (const [x1, z1, x2, z2] of walls) {
      const sx = x2 - x1, sz = z2 - z1;
      const denominator = cross(dx, dz, sx, sz);
      if (Math.abs(denominator) < 1e-10) continue;
      const qx = x1 - pose.x, qz = z1 - pose.z;
      const t = cross(qx, qz, sx, sz) / denominator;
      const u = cross(qx, qz, dx, dz) / denominator;
      if (t >= 0 && u >= 0 && u <= 1) distance = Math.min(distance, t);
    }
    if (distance <= config.range) {
      const std = config.noise ?? 0;
      const noisy = std > 0 ? Math.min(config.range, Math.max(0, distance + gaussian(config.random) * std)) : distance;
      return { angle, distance: noisy, kind: 'wall' };
    }
    // In this ideal flat-floor/vertical-wall laboratory a downward depth return
    // confirms a ground endpoint and the unobstructed 2D column path to it.
    // A missing/invalid return is a distinct observation and NEVER clears cells.
    return config.ground
      ? { angle, distance: config.range, kind: 'floor' }
      : { angle, distance: null, kind: 'invalid' };
  });
}
