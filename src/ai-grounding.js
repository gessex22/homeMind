import { center, FREE } from './mapping.js';
import { footprintClear, routeTo } from './planner.js';

const median = values => [...values].sort((a, b) => a - b)[Math.floor(values.length / 2)];
// Camera-relative measurements are projected using the estimated capture
// pose, never the physical simulator pose or a hidden object inventory.
export function groundDetection(points, belief, { height = .6, forwardOffset = .11, range = 3.8, pitchDeg = 0 } = {}) {
  const usable = points.filter(p => [p.x, p.y, p.z].every(Number.isFinite) && p.z < -.1 && Math.hypot(p.x, p.y, p.z) <= range);
  if (usable.length < 5) return null;
  const depth = median(usable.map(p => -p.z));
  const kept = usable.filter(p => Math.abs(-p.z - depth) <= Math.max(.12, depth * .15));
  if (kept.length < 5) return null;
  const local = { x: median(kept.map(p => p.x)), y: median(kept.map(p => p.y)), z: median(kept.map(p => p.z)) };
  const pitch = pitchDeg * Math.PI / 180, cp = Math.cos(pitch), sp = Math.sin(pitch);
  const y = cp * local.y + sp * local.z, z = -sp * local.y + cp * local.z;
  const s = Math.sin(belief.yaw), c = Math.cos(belief.yaw);
  return { x: belief.x - s * forwardOffset + c * local.x + s * z,
    z: belief.z - c * forwardOffset - s * local.x + c * z, y: height + y,
    depth, samples: kept.length, spread: Math.max(...kept.map(p => -p.z)) - Math.min(...kept.map(p => -p.z)) };
}
export function faceObjectCommand(belief, object) {
  const angle = Math.atan2(-(object.x - belief.x), -(object.z - belief.z));
  const delta = Math.atan2(Math.sin(angle - belief.yaw), Math.cos(angle - belief.yaw));
  return Math.abs(delta) < .06 ? null : { left: delta > 0, right: delta < 0 };
}
export function approachForObject(nav, object, radius) {
  const candidates = [];
  for (let i = 0; i < nav.map.length; i++) if (nav.map[i] === FREE) {
    const p = center(i, nav.map), distance = Math.hypot(p.x - object.x, p.z - object.z);
    if (distance < .55 || distance > 1.25 || !footprintClear(nav.map, p, radius)) continue;
    candidates.push({ ...p, score: Math.abs(distance - .8) * 3 + Math.hypot(p.x - nav.belief.x, p.z - nav.belief.z) });
  }
  candidates.sort((a, b) => a.score - b.score);
  for (const p of candidates.slice(0, 20)) {
    const route = routeTo(nav.map, nav.belief, p, { radius });
    if (route !== null) return { x: p.x, z: p.z, yaw: Math.atan2(-(object.x - p.x), -(object.z - p.z)) };
  }
  return null;
}
