import { readFileSync } from 'node:fs';
import { extractGlbGeometry } from '../src/glb-geometry.js';
import { createScanSession } from '../src/scan-session.js';
import { huellaTransitable } from '../src/scan-world.js';
const args = process.argv.slice(2), i = args.indexOf('--calibration');
const calibration = i >= 0 ? JSON.parse(readFileSync(args[i + 1], 'utf8')) : null;
const glbArg = args.find(a => a.endsWith('.glb'));
const geometry = extractGlbGeometry(readFileSync(glbArg ?? new URL('../public/scans/polycam-2026-10-28/10_28_2026.glb', import.meta.url)));
const session = createScanSession(geometry, { scale: calibration?.scale ?? 1, seed: 7, noisy: true });
session.nav.start(); let reason = 'limite_de_120_s';
for (let j = 0; j < 7200; j++) {
  const result = session.step();
  if (!huellaTransitable(session.world, session.pose, session.world.radius)) throw new Error('La huella salió de la superficie admitida');
  if (result.stopped) { reason = result.groundStop ? 'suelo_no_verificado' : session.nav.state; break; }
}
console.log(JSON.stringify({ reason, calibration, ...session.diagnostics, geometry: session.world.stats, navigation: session.nav.summary() }, null, 2));
