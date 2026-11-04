import { alignScan, blendPose, SCANMATCH } from './scanmatch.js';
import { assessLocalization, observeLocalization, invalidateLocalization } from './localization.js';
import { integrate } from './mapping.js';
import { projectScan, SENSOR } from './sensor.js';
import { createSlam, observeSlam, slamSummary } from './slam.js';
import { between, compose } from './pose-graph.js';
import { createMotionFusion, observeLidarMotion, consumeLidarStall } from './motion-fusion.js';

export function createMissionMapping(map, { noisy = true, loopClosure = true, motionFusion = true, scanmatch = {}, initialPose = { x: 0, z: 0, yaw: 0 } } = {}) {
  return { noisy, loopClosure, scanmatch, motion: createMotionFusion({ enabled: noisy && motionFusion }), slam: noisy && loopClosure ? createSlam(map, initialPose) : null, corrections: 0, correctionDistance: 0 };
}
export function updateMissionMapping(mission, map, rays, estimated, explorer, time, sensor = SENSOR, contactPending = false) {
  const lidar = observeLidarMotion(mission.motion, rays, estimated, sensor);
  let pose = lidar.pose;
  if (lidar.suspect) return { pose, rays: projectScan(rays, pose, sensor), loop: { pose, rebuilt: false }, lidar, deferred: true };
  if (lidar.corrected) {
    invalidateLocalization(explorer.localizacion);
    explorer.route = []; explorer.safe = null; explorer.safeRevision = -1;
    if (consumeLidarStall(mission.motion) && ['barrido', 'navegando', 'planificando', 'retorno', 'retroceso', 'relocalizando'].includes(explorer.state)) {
      explorer.state = 'relocalizando'; explorer.relocalizeAngle = 0;
    }
  }
  if (mission.noisy && !mission.motion.blockedDirection && !lidar.corrected) {
    const fix = alignScan(map, rays, pose, sensor, mission.scanmatch);
    if (fix.corregido) {
      const corrected = blendPose(pose, fix.pose, mission.scanmatch.alpha ?? SCANMATCH.alpha);
      mission.corrections++; mission.correctionDistance += Math.hypot(corrected.x - pose.x, corrected.z - pose.z); pose = corrected;
    }
  }
  const loop = mission.slam ? observeSlam(mission.slam, map, rays, pose, sensor, time, !contactPending && !mission.motion.blockedDirection && explorer.state !== 'retroceso') : { pose, rebuilt: false };
  pose = loop.pose;
  if (loop.rebuilt) {
    // Scan matching and route cache refer to the old geometry. Replan before
    // translating, and move visited scan positions with their nearest anchor.
    explorer.scanned = explorer.scanned.map(p => {
      let nearest = 0, distance = Infinity;
      loop.before.forEach((old, i) => { const d = Math.hypot(old.x - p.x, old.z - p.z); if (d < distance) { distance = d; nearest = i; } });
      return compose(loop.after[nearest], between(loop.before[nearest], { ...p, yaw: p.yaw ?? 0 }));
    });
    explorer.route = []; explorer.safe = null; explorer.safeRevision = -1;
    invalidateLocalization(explorer.localizacion);
    if (['barrido', 'navegando', 'planificando', 'retorno', 'retroceso', 'relocalizando'].includes(explorer.state)) {
      explorer.state = 'relocalizando'; explorer.relocalizeAngle = 0;
    }
  }
  // Evaluate only pre-insertion evidence; after a replay, this scan is already
  // in the map and cannot be used to confirm itself.
  if (!loop.rebuilt && !lidar.corrected) observeLocalization(explorer.localizacion, assessLocalization(map, rays, pose, sensor), pose);
  const projected = projectScan(rays, pose, sensor);
  if (!loop.rebuilt) integrate(map, pose, projected);
  return { pose, rays: projected, loop, lidar };
}
export function missionSummary(mission) {
  return mission.slam ? slamSummary(mission.slam) : { status: mission.noisy ? 'desactivado' : 'control_ideal', scans: 0, submaps: 0, nodes: 0, closures: 0, candidates: 0, rejected: 0, confirmations: 0, events: [], limited: false };
}
