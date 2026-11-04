import { cell, center, FREE } from './mapping.js';
import { footprintClear, routeTo } from './planner.js';
import { routeAfterRecovery } from './explorer.js';
import { t } from './i18n.js';

// Canvas uses a square bitmap with object-fit: contain. Exclude letterboxing
// before converting a pointer into coordinates in the robot's map frame.
export function destinationFromPointer(clientX, clientY, rect, grid) {
  const side = Math.min(rect.width, rect.height);
  if (!(side > 0)) return null;
  const x = (clientX - rect.left - (rect.width - side) / 2) * 640 / side;
  const z = (clientY - rect.top - (rect.height - side) / 2) * 640 / side;
  const unit = 640 / Math.max(grid.width, grid.height), ox = (640 - grid.width * unit) / 2, oz = (640 - grid.height * unit) / 2;
  if (x < ox || z < oz || x >= ox + grid.width * unit || z >= oz + grid.height * unit) return null;
  return { x: grid.originX + (x - ox) / unit * grid.resolution, z: grid.originZ + (z - oz) / unit * grid.resolution };
}

// Only the discovered map is consulted. Selecting an observer reference
// cannot expose unobserved geometry to the route planner.
export function selectMapDestination(nav, point, radius) {
  if (!point || !Number.isFinite(point.x) || !Number.isFinite(point.z)) throw new Error(t('destinoFuera'));
  const index = cell(point.x, point.z, nav.map);
  if (index < 0 || nav.map[index] !== FREE) throw new Error(t('destinoNoDescubierto'));
  const goal = center(index, nav.map);
  if (!footprintClear(nav.map, goal, radius)) throw new Error(t('destinoNoCabe'));
  const options = { radius, avoid: nav.explorer?.avoid };
  let route = routeTo(nav.map, nav.belief, goal, options);
  if (route === null) {
    // Prove both a safe exit from the current wall and a connected route;
    // that exit can require turning before moving instead of reversing.
    const recovery = routeAfterRecovery(nav.map, nav.belief, goal, options);
    if (recovery) route = [cell(recovery.pose.x, recovery.pose.z, nav.map), ...recovery.route];
  }
  if (route === null) throw new Error(t('destinoSinRuta'));
  nav.start(); nav.setDestination(goal);
  return { goal, route };
}
