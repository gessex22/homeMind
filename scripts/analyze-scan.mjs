import fs from 'node:fs';
import { extractGlbGeometry } from '../src/glb-geometry.js';
import { buildScanWorld } from '../src/scan-world.js';
const input = process.argv[2] ?? 'public/scans/polycam-2026-10-28/10_28_2026.glb';
const geometry = extractGlbGeometry(fs.readFileSync(input));
const start = performance.now(), world = buildScanWorld(geometry);
console.log(JSON.stringify({ ...world.stats, initialPose: world.initialPose, grid: world.grid, ms: Math.round(performance.now() - start) }, null, 2));
for (let z = 0; z < world.grid.height; z += 2) {
  let row = '';
  for (let x = 0; x < world.grid.width; x += 2) row += [' ', '.', '#'][world.map[z * world.grid.width + x]];
  console.log(row);
}
