// Robot-side inventory: fuses repeated color detections into one entry per
// physical board. Each observation is located by bearing plus apparent size
// (the marker edge is shared prior knowledge, like the beam layout); same
// label within RADIO clusters, anything farther splits. Uses belief poses,
// never ground truth: drift shifts entries like it shifts the map.
export const TABLERO_M = .48;
export const RADIO_FUSION_M = .75;

export function locateDetection(frame, detection) {
  const pose = frame?.pose, cam = frame?.calibracion, box = detection?.bbox;
  if (!pose || !cam || !box) return null;
  if (![pose.x, pose.z, pose.yaw, cam.fx, cam.cx, box.x, box.width].every(Number.isFinite)) return null;
  if (!(cam.fx > 0) || !(box.width > 0)) return null;
  const beta = Math.atan((box.x + box.width / 2 - cam.cx) / cam.fx);
  const dist = cam.fx * TABLERO_M / box.width;
  const right = dist * Math.sin(beta), depth = dist * Math.cos(beta);
  const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
  return { x: pose.x + c * right - s * depth, z: pose.z - s * right - c * depth, dist };
}

export function fuseInventory(keyframes, { radio = RADIO_FUSION_M } = {}) {
  if (!Number.isFinite(radio) || radio <= 0) throw new RangeError('El radio de fusión debe ser positivo.');
  const entries = [];
  for (const frame of keyframes ?? []) {
    for (const det of frame?.detections ?? []) {
      if (typeof det?.label !== 'string' || !det.label) continue;
      const at = locateDetection(frame, det);
      if (!at) continue;
      const area = Number.isFinite(det.area) ? det.area : 0;
      const score = area * (det.frontal === true ? 2 : 1);
      const seen = entries.filter(e => e.label === det.label).length;
      const entry = entries.find(e => e.label === det.label && Math.hypot(e.x - at.x, e.z - at.z) < radio)
        ?? { id: `${det.label}-${seen + 1}`, label: det.label, x: at.x, z: at.z, observaciones: 0, frontales: 0, mejor_score: -1, mejor_keyframe: null, keyframes: [], primera_vista_s: frame.tiempo_s, ultima_vista_s: frame.tiempo_s };
      if (!entries.includes(entry)) entries.push(entry);
      entry.observaciones += 1;
      if (det.frontal === true) entry.frontales += 1;
      entry.x += (at.x - entry.x) / entry.observaciones;
      entry.z += (at.z - entry.z) / entry.observaciones;
      if (score > entry.mejor_score) { entry.mejor_score = score; entry.mejor_keyframe = frame.id; }
      if (!entry.keyframes.includes(frame.id)) entry.keyframes.push(frame.id);
      entry.ultima_vista_s = frame.tiempo_s;
    }
  }
  return entries;
}
