// Angular coverage comes from received scans and estimated poses, not commands
// or simulator geometry. Conservative 2-degree bins avoid counting unseen gaps.
const TAU = Math.PI * 2, BINS = 180, BIN = TAU / BINS;
const wrap = angle => Math.atan2(Math.sin(angle), Math.cos(angle));
export function observeSweep(coverage, pose, fov) {
  if (!coverage || Math.hypot(pose.x - coverage.x, pose.z - coverage.z) > .03) {
    coverage = { x: pose.x, z: pose.z, seen: new Uint8Array(BINS), count: 0 };
  }
  if (!Number.isFinite(fov) || fov <= 0) return coverage;
  for (let i = 0; i < BINS; i++) {
    if (!coverage.seen[i] && Math.abs(wrap((i + .5) * BIN - pose.yaw)) <= fov / 2 - BIN / 2 + 1e-10) {
      coverage.seen[i] = 1; coverage.count++;
    }
  }
  return coverage;
}
export const sweepCovered = coverage => coverage?.count === BINS;
