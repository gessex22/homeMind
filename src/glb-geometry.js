// Geometry-only GLB reader for the simulator adapter and offline validation.
// Textures are left to GLTFLoader. Unsupported encodings fail explicitly.
import { Matrix4, Vector3, Quaternion } from 'three';

export function extractGlbGeometry(input) {
  const bytes = input instanceof ArrayBuffer ? new Uint8Array(input) : new Uint8Array(input.buffer, input.byteOffset, input.byteLength);
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  if (bytes.length < 20 || view.getUint32(0, true) !== 0x46546c67 || view.getUint32(4, true) !== 2 || view.getUint32(8, true) !== bytes.length) throw new Error('GLB 2 inválido');
  let json, binary;
  for (let offset = 12; offset < bytes.length;) {
    const length = view.getUint32(offset, true), type = view.getUint32(offset + 4, true);
    if (offset + 8 + length > bytes.length) throw new Error('Chunk GLB fuera de rango');
    const data = bytes.subarray(offset + 8, offset + 8 + length);
    if (type === 0x4e4f534a) json = JSON.parse(new TextDecoder().decode(data));
    if (type === 0x004e4942) binary = data;
    offset += 8 + length;
  }
  if (!json || !binary || json.buffers?.some(b => b.uri) || json.extensionsRequired?.length) throw new Error('La captura requiere recursos o extensiones no admitidos');
  const bin = new DataView(binary.buffer, binary.byteOffset, binary.byteLength);
  function accessor(index) {
    const a = json.accessors[index], b = json.bufferViews[a?.bufferView];
    const sizes = { 5121: 1, 5123: 2, 5125: 4, 5126: 4 }, components = { SCALAR: 1, VEC3: 3 };
    if (!a || !b || a.sparse || b.buffer !== 0 || !sizes[a.componentType] || !components[a.type]) throw new Error('Accessor geométrico no admitido');
    const offset = (b.byteOffset ?? 0) + (a.byteOffset ?? 0), stride = b.byteStride ?? sizes[a.componentType] * components[a.type];
    if (offset + (a.count - 1) * stride + sizes[a.componentType] * components[a.type] > binary.length) throw new Error('Accessor fuera del binario');
    return { count: a.count, get(i, c = 0) {
      if (i < 0 || i >= a.count) throw new Error('Índice geométrico inválido');
      const o = offset + i * stride + c * sizes[a.componentType];
      return a.componentType === 5126 ? bin.getFloat32(o, true) : a.componentType === 5125 ? bin.getUint32(o, true) : a.componentType === 5123 ? bin.getUint16(o, true) : bin.getUint8(o);
    } };
  }
  const values = [], point = new Vector3(), bounds = { minX: Infinity, minY: Infinity, minZ: Infinity, maxX: -Infinity, maxY: -Infinity, maxZ: -Infinity };
  function visit(index, parent, ancestors) {
    if (ancestors.has(index)) throw new Error('Ciclo de nodos GLB');
    const node = json.nodes[index]; if (!node) throw new Error('Nodo GLB inválido');
    const local = node.matrix ? new Matrix4().fromArray(node.matrix) : new Matrix4().compose(new Vector3(...(node.translation ?? [0, 0, 0])), new Quaternion(...(node.rotation ?? [0, 0, 0, 1])), new Vector3(...(node.scale ?? [1, 1, 1])));
    const matrix = parent.clone().multiply(local);
    if (node.mesh != null) for (const primitive of json.meshes[node.mesh].primitives) {
      if ((primitive.mode ?? 4) !== 4) throw new Error('Se requieren triángulos');
      const positions = accessor(primitive.attributes.POSITION), indices = primitive.indices == null ? null : accessor(primitive.indices), count = indices?.count ?? positions.count;
      if (count % 3) throw new Error('Triangulación incompleta');
      for (let i = 0; i < count; i++) {
        const k = indices ? indices.get(i) : i;
        point.set(positions.get(k, 0), positions.get(k, 1), positions.get(k, 2)).applyMatrix4(matrix);
        if (![point.x, point.y, point.z].every(Number.isFinite)) throw new Error('Coordenada no finita');
        values.push(point.x, point.y, point.z);
        bounds.minX = Math.min(bounds.minX, point.x); bounds.maxX = Math.max(bounds.maxX, point.x);
        bounds.minY = Math.min(bounds.minY, point.y); bounds.maxY = Math.max(bounds.maxY, point.y);
        bounds.minZ = Math.min(bounds.minZ, point.z); bounds.maxZ = Math.max(bounds.maxZ, point.z);
      }
    }
    const next = new Set(ancestors).add(index);
    for (const child of node.children ?? []) visit(child, matrix, next);
  }
  for (const node of json.scenes[json.scene ?? 0].nodes) visit(node, new Matrix4(), new Set());
  if (!values.length) throw new Error('Captura sin triángulos');
  const cx = (bounds.minX + bounds.maxX) / 2, cz = (bounds.minZ + bounds.maxZ) / 2;
  for (let i = 0; i < values.length; i += 3) { values[i] -= cx; values[i + 1] -= bounds.minY; values[i + 2] -= cz; }
  return { triangles: new Float32Array(values), originalBounds: bounds,
    bounds: { minX: bounds.minX - cx, maxX: bounds.maxX - cx, minY: 0, maxY: bounds.maxY - bounds.minY, minZ: bounds.minZ - cz, maxZ: bounds.maxZ - cz } };
}
