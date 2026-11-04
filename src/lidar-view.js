import { projectScan, SENSOR } from './sensor.js';

// Sensor bearings are reconstructed from calibration; raw simulator angles
// must not be combined with belief yaw (or leaked into the believed map).
export function lidarViewRays(scan) {
  const pose = scan.belief;
  return projectScan(scan.rays, pose, scan.sensor ?? SENSOR)
    .filter(ray => ray.kind !== 'invalid' && Number.isFinite(ray.distance) && ray.distance > 0)
    .map(ray => ({ ...ray, x: pose.x - Math.sin(ray.angle) * ray.distance,
      z: pose.z - Math.cos(ray.angle) * ray.distance }));
}
