import { predictMotion, STEP } from './motion.js';
import { SCENARIOS } from './world.js';
export { SPEED, TURN_SPEED, STEP } from './motion.js';

export const ROOM_HALF = 3;
export const ROBOT_RADIUS = 0.25;

function distanceToSegment(x, z, [x1, z1, x2, z2]) {
  const dx = x2 - x1, dz = z2 - z1, length2 = dx * dx + dz * dz;
  const t = length2 ? Math.max(0, Math.min(1, ((x - x1) * dx + (z - z1) * dz) / length2)) : 0;
  return Math.hypot(x - x1 - t * dx, z - z1 - t * dz);
}
export function advance(pose, controls, dt, walls = SCENARIOS.empty.walls) {
  const { x: nextX, z: nextZ, yaw } = predictMotion(pose, controls, dt);
  const dx = nextX - pose.x, dz = nextZ - pose.z;
  const steps = Math.max(1, Math.ceil(Math.hypot(dx, dz) / .01));
  let x = pose.x, z = pose.z, collision = false;
  for (let i = 0; i < steps; i++) {
    const nx = x + dx / steps, nz = z + dz / steps;
    const blocked = walls.some(wall => {
      const before = distanceToSegment(x, z, wall), after = distanceToSegment(nx, nz, wall);
      return after < ROBOT_RADIUS - 1e-9 && after < before - 1e-10;
    });
    if (blocked) {
      let lo = 0, hi = 1;
      for (let j = 0; j < 32; j++) {
        const mid = (lo + hi) / 2, mx = x + dx / steps * mid, mz = z + dz / steps * mid;
        if (walls.some(wall => distanceToSegment(mx, mz, wall) < ROBOT_RADIUS)) hi = mid; else lo = mid;
      }
      x = Math.round((x + dx / steps * lo) * 1e10) / 1e10;
      z = Math.round((z + dz / steps * lo) * 1e10) / 1e10;
      collision = true; break;
    }
    x = nx; z = nz;
  }
  return { x: collision ? x : nextX, z: collision ? z : nextZ, yaw, collision };
}

export function sample(pose, step, startTime) {
  return { paso: step, timestamp: new Date(startTime + step * STEP * 1000).toISOString(), x: pose.x, z: pose.z, yaw_rad: pose.yaw };
}
