// Kinematics only: no simulator geometry or collision ground truth.
export const SPEED = 1;
export const TURN_SPEED = Math.PI / 2;
export const STEP = 1 / 60;
export const WHEELBASE = .3;

// Booleans remain full-speed manual controls; autonomous controls may use
// finite fractions. Invalid values are treated as zero.
export function controlValue(value) {
  if (value === true) return 1;
  if (typeof value !== 'number' || !Number.isFinite(value)) return 0;
  return Math.max(0, Math.min(1, value));
}
export function driveValue(command) { return controlValue(command.forward) - controlValue(command.back); }
export function turnValue(command) { return controlValue(command.left) - controlValue(command.right); }

export function wheelSpeeds(command, wheelbase = WHEELBASE) {
  const drive = driveValue(command);
  const turn = turnValue(command);
  const v = drive * SPEED, w = turn * TURN_SPEED;
  return { izq: v - w * wheelbase / 2, der: v + w * wheelbase / 2, avance: v, giro: w };
}

export function predictMotion(pose, controls, dt) {
  const turn = turnValue(controls);
  const drive = driveValue(controls);
  const angle = pose.yaw + turn * TURN_SPEED * dt;
  const yaw = Math.atan2(Math.sin(angle), Math.cos(angle));
  return {
    x: pose.x - Math.sin(yaw) * drive * SPEED * dt,
    z: pose.z - Math.cos(yaw) * drive * SPEED * dt,
    yaw,
  };
}
