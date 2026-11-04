import { createNavigator } from './navigation.js';
import { buildScanWorld, advanceScanWorld, senseScanWorld } from './scan-world.js';
import { createOdometry, readMotion, SENSORES_RUIDOSOS } from './odometry.js';
import { SENSOR } from './sensor.js';
import { mulberry32 } from './noise.js';
import { STEP } from './motion.js';
import { markBump } from './mapping.js';

// Host-side simulator: only rays and measured motion cross into the engine.
export function createScanSession(geometry, { scale = 1, noisy = true, seed = 7 } = {}) {
  const world = buildScanWorld(geometry, { scale });
  if (!world.initialPose) throw new Error('No hay suelo capturado suficiente para una huella de 20 cm');
  const nav = createNavigator(world.grid, { initialPose: world.initialPose, noisy, seed, footprint: world.radius, adaptiveSweeps: true });
  const adapter = createOdometry(world.initialPose, noisy ? { ...SENSORES_RUIDOSOS, seed } : { seed });
  const sensor = { ...SENSOR, noise: noisy ? .01 : 0, random: mulberry32(seed ^ 0x51f3) };
  let pose = { ...world.initialPose }, lastMotion = null, steps = 0, contacts = 0, groundStops = 0, distance = 0;
  let lastScan = null, lastCommand = null;
  const trail = [{ ...pose }];
  return { world, nav, trail,
    get pose() { return { ...pose }; },
    get lastScan() { return lastScan; },
    get lastCommand() { return lastCommand; },
    get diagnostics() { return { steps, seconds: steps * STEP, distance, contacts, groundStops, positionError: Math.hypot(pose.x - nav.belief.x, pose.z - nav.belief.z) }; },
    step(manualCommand = null) {
      if (manualCommand !== null) {
        if (nav.state === 'inactivo') nav.start();
        nav.pause();
      }
      const rays = steps % 6 === 0 ? senseScanWorld(world, pose, sensor) : null;
      const auto = nav.step(STEP, { rays, motion: lastMotion });
      if (rays) lastScan = { rays, belief: { ...nav.belief }, sensor: { fov: sensor.fov, range: sensor.range, rays: sensor.rays } };
      const command = manualCommand === null ? auto : Object.fromEntries(['forward', 'back', 'left', 'right'].map(k => [k, Boolean(manualCommand[k])]));
      lastCommand = command;
      const next = advanceScanWorld(world, pose, command, STEP);
      lastMotion = { ...readMotion(adapter, next, pose, command, STEP, next.collision), groundStop: Boolean(next.groundStop) };
      distance += Math.hypot(next.x - pose.x, next.z - pose.z); steps++;
      if (next.collision) {
        contacts++;
        if (manualCommand !== null) markBump(nav.map, nav.belief, command.back && !command.forward, world.radius);
      }
      // A floor interlock is a sensor event the autonomous controller can
      // recover from. Manual driving still stops immediately at the edge.
      if (next.groundStop) { groundStops++; if (manualCommand !== null) nav.pause(); }
      pose = next;
      if (Math.hypot(pose.x - trail.at(-1).x, pose.z - trail.at(-1).z) >= .03) trail.push({ ...pose });
      return { stopped: (next.groundStop && manualCommand !== null) || (manualCommand === null && nav.terminal), groundStop: next.groundStop };
    },
  };
}
