// Select observations from motion and time only. No scene, objects or simulator geometry.
export const KEYFRAME_DEFAULTS = Object.freeze({
  translation: .1,
  rotation: Math.PI / 18,
  minInterval: .15,
  maxFrames: 400,
});

export function createKeyframeSelector(options = {}) {
  const config = { ...KEYFRAME_DEFAULTS, ...options };
  if (!Number.isFinite(config.translation) || config.translation <= 0
    || !Number.isFinite(config.rotation) || config.rotation <= 0
    || !Number.isFinite(config.minInterval) || config.minInterval < 0
    || !Number.isInteger(config.maxFrames) || config.maxFrames <= 0) {
    throw new RangeError('Los umbrales de keyframes deben ser positivos y el límite un entero.');
  }
  return { config, count: 0, last: null };
}

export function selectKeyframe(selector, pose, time, options = {}) {
  if (![pose.x, pose.z, pose.yaw, time].every(Number.isFinite)) {
    throw new TypeError('La captura requiere una pose y un tiempo finitos.');
  }
  if (selector.count >= selector.config.maxFrames) return { capture: false, reason: 'limite' };
  const previous = selector.last;
  if (previous && time - previous.time < selector.config.minInterval - 1e-9) {
    return { capture: false, reason: 'espera' };
  }

  const distance = previous ? Math.hypot(pose.x - previous.x, pose.z - previous.z) : 0;
  const angle = previous
    ? Math.abs(Math.atan2(Math.sin(pose.yaw - previous.yaw), Math.cos(pose.yaw - previous.yaw)))
    : 0;
  let reason;
  if (options.force) reason = 'manual';
  else if (!previous) reason = 'inicial';
  else if (distance >= selector.config.translation - 1e-9) reason = 'traslacion';
  else if (angle >= selector.config.rotation - 1e-9) reason = 'giro';
  else return { capture: false, reason: 'espera' };

  selector.last = { x: pose.x, z: pose.z, yaw: pose.yaw, time };
  selector.count++;
  return { capture: true, reason, sequence: selector.count };
}
