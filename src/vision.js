// Pure image-based detector; it has no access to world geometry or scene IDs.
function classify(r, g, b) {
  const max = Math.max(r, g, b), min = Math.min(r, g, b);
  if (max < 65 || (max - min) / max < .38) return 0;
  if (r > 80 && g > 65 && b < Math.min(r, g) * .60 && r / g < 2.4 && g / r < 1.5) return 3;
  if (r > g * 1.45 && r > b * 1.45) return 1;
  if (b > r * 1.45 && b > g * 1.35) return 2;
  return 0;
}
export function detectMarkers({ data, width, height }) {
  if (!data || data.length !== width * height * 4 || width < 1 || height < 1) throw new Error('Imagen RGBA inválida');
  const labels = ['', 'rojo', 'azul', 'amarillo'], classes = new Uint8Array(width * height), seen = new Uint8Array(width * height);
  for (let i = 0; i < classes.length; i++) classes[i] = classify(data[i * 4], data[i * 4 + 1], data[i * 4 + 2]);
  const detections = [];
  for (let i = 0; i < classes.length; i++) {
    const color = classes[i]; if (!color || seen[i]) continue;
    const queue = [i]; seen[i] = 1; let minX = width, maxX = 0, minY = height, maxY = 0;
    for (let q = 0; q < queue.length; q++) {
      const p = queue[q], x = p % width, y = Math.floor(p / width);
      minX = Math.min(minX, x); maxX = Math.max(maxX, x); minY = Math.min(minY, y); maxY = Math.max(maxY, y);
      if (x > 0) add(p - 1); if (x + 1 < width) add(p + 1); if (y > 0) add(p - width); if (y + 1 < height) add(p + width);
      function add(n) { if (!seen[n] && classes[n] === color) { seen[n] = 1; queue.push(n); } }
    }
    const area = queue.length; if (area < 30) continue;
    const w = maxX - minX + 1, h = maxY - minY + 1, clipped = minX <= 1 || minY <= 1 || maxX >= width - 2 || maxY >= height - 2;
    const frontal = !clipped && w >= 12 && h >= 12 && w / h >= .7 && w / h <= 1.4 && area / (w * h) >= .7;
    detections.push({ label: labels[color], bbox: { x: minX, y: minY, width: w, height: h }, area, frontal, clipped, lado_min_px: Math.min(w, h), proporcion_ancho_imagen: Math.min(w, h) / width });
  }
  return detections.sort((a, b) => b.area - a.area);
}
