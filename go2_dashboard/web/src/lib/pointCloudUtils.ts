import type { PointCloud2Message } from './rosMessages';

const FLOAT32 = 7;

export interface ParsedPointCloud {
  positions: Float32Array;
  colors: Float32Array;
  count: number;
  frameId: string;
}

function decodeData(data: number[] | string): Uint8Array {
  if (typeof data === 'string') {
    return Uint8Array.from(atob(data), (c) => c.charCodeAt(0));
  }
  return Uint8Array.from(data);
}

function readField(
  view: DataView,
  offset: number,
  datatype: number,
  littleEndian: boolean,
): number {
  switch (datatype) {
    case FLOAT32:
      return view.getFloat32(offset, littleEndian);
  }
  return Number.NaN;
}

function heightColor(z: number): [number, number, number] {
  const t = Math.max(0, Math.min(1, (z + 0.5) / 3));
  const r = 0.25 + t * 0.55;
  const g = 0.55 + (1 - Math.abs(t - 0.5) * 2) * 0.25;
  const b = 0.95 - t * 0.65;
  return [r, g, b];
}

export function parsePointCloud2(
  msg: PointCloud2Message,
  maxPoints = 120_000,
): ParsedPointCloud | null {
  const { width, height, fields, point_step, data, is_bigendian, header } = msg;
  const total = width * height;
  if (!total || !data || !point_step) return null;

  const xField = fields.find((f) => f.name === 'x');
  const yField = fields.find((f) => f.name === 'y');
  const zField = fields.find((f) => f.name === 'z');
  if (!xField || !yField || !zField) return null;

  const bytes = decodeData(data);
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const littleEndian = !is_bigendian;
  const stride = Math.max(1, Math.ceil(total / maxPoints));

  const positions = new Float32Array(maxPoints * 3);
  const colors = new Float32Array(maxPoints * 3);
  let count = 0;

  for (let i = 0; i < total && count < maxPoints; i += stride) {
    const base = i * point_step;
    if (base + point_step > bytes.length) break;

    const x = readField(view, base + xField.offset, xField.datatype, littleEndian);
    const y = readField(view, base + yField.offset, yField.datatype, littleEndian);
    const z = readField(view, base + zField.offset, zField.datatype, littleEndian);

    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;
    if (Math.abs(x) > 500 || Math.abs(y) > 500 || Math.abs(z) > 50) continue;

    const idx = count * 3;
    positions[idx] = x;
    positions[idx + 1] = y;
    positions[idx + 2] = z;

    const [r, g, b] = heightColor(z);
    colors[idx] = r;
    colors[idx + 1] = g;
    colors[idx + 2] = b;
    count += 1;
  }

  if (!count) return null;

  return {
    positions: positions.subarray(0, count * 3),
    colors: colors.subarray(0, count * 3),
    count,
    frameId: header.frame_id || 'base_link',
  };
}
