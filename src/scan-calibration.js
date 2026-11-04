export const CALIBRATION_KEY = 'homemind:polycam-2026-10-28:scale:v1';
export function tileCalibration(a, b, referenceCm = 32.7) {
  const measuredCaptureM = Math.hypot(a.x - b.x, a.z - b.z);
  const scale = referenceCm / 100 / measuredCaptureM;
  if (!Number.isFinite(scale) || scale < .25 || scale > 4 || Math.abs(a.y - b.y) > .05) throw new Error('Selecciona los dos extremos de un lado completo de la misma baldosa');
  return { asset: '10_28_2026.glb', referenceCm, measuredCaptureM, scale, endpoints: [a, b], method: 'Dos puntos sobre baldosa sin junta', timestamp: new Date().toISOString() };
}
export function readCalibration() {
  try {
    const c = JSON.parse(localStorage.getItem(CALIBRATION_KEY));
    return c?.asset === '10_28_2026.glb' && Number.isFinite(c.scale) && c.scale >= .25 && c.scale <= 4 ? c : null;
  } catch { return null; }
}
