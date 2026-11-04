// Observer-only grading. Never feed these results, board positions or counts
// into the image detector, keyframe selector or autonomous exploration.
const CAMERA_HEIGHT = .35;
const MAX_FRONT_ANGLE = 35 * Math.PI / 180;

function gradeView(board, frame, detection) {
  const { calibracion: camera } = frame;
  const pose = frame.pose_real ?? frame.pose;
  if (!pose || !camera || !detection.bbox || detection.frontal !== true || detection.clipped) return null;
  if (![pose.x, pose.z, pose.yaw, camera.width, camera.height, camera.fx, camera.fy, camera.cx, camera.cy]
    .every(Number.isFinite) || camera.width <= 0 || camera.height <= 0 || camera.fx <= 0 || camera.fy <= 0) return null;
  if (typeof frame.imagen_png !== 'string' || !frame.imagen_png.startsWith('data:image/png;base64,')) return null;

  const dx = board.x - pose.x, dz = board.z - pose.z;
  const distance = Math.hypot(dx, dz);
  if (!Number.isFinite(distance) || distance < 1e-8 || !Number.isFinite(board.yaw)) return null;
  // PlaneGeometry faces +Z before rotation around Y.
  const facing = (-dx * Math.sin(board.yaw) - dz * Math.cos(board.yaw)) / distance;
  if (facing < Math.cos(MAX_FRONT_ANGLE) - 1e-8) return null;
  const depth = -dx * Math.sin(pose.yaw) - dz * Math.cos(pose.yaw);
  if (depth <= 0) return null;
  const right = dx * Math.cos(pose.yaw) - dz * Math.sin(pose.yaw);
  const u = camera.cx + camera.fx * right / depth;
  const v = camera.cy - camera.fy * ((board.y ?? CAMERA_HEIGHT) - CAMERA_HEIGHT) / depth;
  const box = detection.bbox;
  if (![box.x, box.y, box.width, box.height].every(Number.isFinite) || box.width <= 0 || box.height <= 0) return null;
  if (u < 0 || u >= camera.width || v < 0 || v >= camera.height
    || u < box.x - 2 || u > box.x + box.width + 2
    || v < box.y - 2 || v > box.y + box.height + 2) return null;
  return Math.acos(Math.max(-1, Math.min(1, facing))) * 180 / Math.PI;
}

export function evaluatePerception(boards, keyframes, { minLadoReferencia = 48 } = {}) {
  if (!Number.isFinite(minLadoReferencia) || minLadoReferencia <= 0) throw new RangeError('El umbral de detalle debe ser positivo.');
  const labelCounts = new Map();
  for (const board of boards) labelCounts.set(board.label, (labelCounts.get(board.label) ?? 0) + 1);
  const ambiguous = [...labelCounts].filter(([, count]) => count > 1).map(([label]) => label);
  const results = boards.map(board => {
    let observation = null;
    if (!ambiguous.includes(board.label)) {
      for (let i = 0; i < keyframes.length; i++) {
        const frame = keyframes[i];
        for (const detection of frame.detections ?? []) {
          if (detection.label !== board.label) continue;
          const angle = gradeView(board, frame, detection);
          if (angle !== null) {
            const side = Math.min(detection.bbox.width, detection.bbox.height);
            const ratio = side / frame.calibracion.width;
            if (!observation || ratio > observation.proporcion_ancho_imagen || (ratio === observation.proporcion_ancho_imagen && angle < observation.angulo_exacto)) {
              observation = { keyframe: i + 1, tiempo_s: frame.tiempo_s, angulo_grados: Number(angle.toFixed(1)), angulo_exacto: angle, lado_min_px: side, proporcion_ancho_imagen: ratio, detalle_suficiente: ratio >= minLadoReferencia / 480 };
            }
          }
        }
      }
    }
    if (observation) delete observation.angulo_exacto;
    return { label: board.label, observado_frontalmente: Boolean(observation), detalle_suficiente: observation?.detalle_suficiente ?? false, motivo: observation ? (observation.detalle_suficiente ? 'detalle suficiente' : 'pendiente: imagen frontal con poco detalle') : 'sin evidencia frontal verificable', ...observation };
  });
  const pending = results.filter(result => !result.detalle_suficiente).map(result => result.label);
  const frontalCount = results.filter(result => result.observado_frontalmente).length;
  return {
    aprobado: boards.length > 0 && pending.length === 0 && ambiguous.length === 0,
    total: boards.length,
    encontrados: frontalCount,
    conDetalle: results.length - pending.length,
    umbral_lado_px_480: minLadoReferencia,
    pendientes: pending,
    ambiguos: ambiguous,
    tableros: results,
    metodo: `Evaluación del observador: color único, proyección con pose ideal y frontal hasta 35°; detalle mínimo escalado desde ${minLadoReferencia} px a 480 px de ancho. No es inventario espacial.`,
  };
}
