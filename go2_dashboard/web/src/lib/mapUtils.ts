import type { OccupancyGrid, OccupancyGridUpdate } from './rosMessages';

export const MAP_FREE = [238, 242, 248] as const;
export const MAP_OCCUPIED = [32, 36, 48] as const;
export const MAP_UNKNOWN = [72, 78, 92] as const;

export function decodeInt8Array(data: number[] | string): number[] {
  if (Array.isArray(data)) return data;
  if (typeof data === 'string') {
    const raw = Uint8Array.from(atob(data), (c) => c.charCodeAt(0));
    return Array.from(new Int8Array(raw.buffer, raw.byteOffset, raw.byteLength));
  }
  return [];
}

export function normalizeMapMessage(msg: {
  info: OccupancyGrid['info'];
  data: number[] | string;
}): OccupancyGrid {
  const cells = msg.info.width * msg.info.height;
  const data = decodeInt8Array(msg.data);
  if (data.length !== cells) {
    console.warn(`Map data length ${data.length} != ${cells} cells`);
  }
  return { info: msg.info, data };
}

export function occupancyToRgb(value: number): [number, number, number] {
  if (value < 0) return [...MAP_UNKNOWN];
  if (value >= 100) return [...MAP_OCCUPIED];
  const t = value / 100;
  return [
    Math.round(MAP_FREE[0] + (MAP_OCCUPIED[0] - MAP_FREE[0]) * t),
    Math.round(MAP_FREE[1] + (MAP_OCCUPIED[1] - MAP_FREE[1]) * t),
    Math.round(MAP_FREE[2] + (MAP_OCCUPIED[2] - MAP_FREE[2]) * t),
  ];
}

export function mapCacheSignature(
  map: OccupancyGrid | null,
  version: number,
): string {
  if (!map) return '';
  return `${version}:${map.info.width}x${map.info.height}:${map.data.length}`;
}

export function rebuildMapCache(
  map: OccupancyGrid,
  canvas: HTMLCanvasElement,
): void {
  const { info, data } = map;
  if (!info.width || !info.height || !data.length) return;

  canvas.width = info.width;
  canvas.height = info.height;
  const ctx = canvas.getContext('2d');
  if (!ctx) return;

  const img = ctx.createImageData(info.width, info.height);
  const len = Math.min(data.length, info.width * info.height);
  for (let i = 0; i < len; i += 1) {
    const [r, g, b] = occupancyToRgb(data[i]);
    const j = i * 4;
    img.data[j] = r;
    img.data[j + 1] = g;
    img.data[j + 2] = b;
    img.data[j + 3] = 255;
  }
  ctx.putImageData(img, 0, 0);
}

export function applyMapUpdate(
  map: OccupancyGrid,
  update: OccupancyGridUpdate,
): OccupancyGrid {
  const { info, data } = map;
  const patch = decodeInt8Array(update.data);
  const nextData = data.slice();
  const { x, y, width, height } = update;

  for (let row = 0; row < height; row += 1) {
    for (let col = 0; col < width; col += 1) {
      const dst = (y + row) * info.width + (x + col);
      if (dst >= 0 && dst < nextData.length) {
        nextData[dst] = patch[row * width + col];
      }
    }
  }

  return { info, data: nextData };
}

export function yawToQuat(yaw: number) {
  const half = yaw / 2;
  return { x: 0, y: 0, z: Math.sin(half), w: Math.cos(half) };
}

export function quatToYaw(q: { x: number; y: number; z: number; w: number }) {
  return Math.atan2(
    2 * (q.w * q.z + q.x * q.y),
    1 - 2 * (q.y * q.y + q.z * q.z),
  );
}

export function mapToCanvas(
  wx: number,
  wy: number,
  map: OccupancyGrid,
  scale: number,
  offsetX: number,
  offsetY: number,
) {
  const { info } = map;
  const mx = (wx - info.origin.position.x) / info.resolution;
  const my = (wy - info.origin.position.y) / info.resolution;
  return {
    x: mx * scale + offsetX,
    y: (info.height - my) * scale + offsetY,
  };
}

export function canvasToMap(
  cx: number,
  cy: number,
  map: OccupancyGrid,
  scale: number,
  offsetX: number,
  offsetY: number,
) {
  const { info } = map;
  const mx = (cx - offsetX) / scale;
  const my = info.height - (cy - offsetY) / scale;
  return {
    x: info.origin.position.x + mx * info.resolution,
    y: info.origin.position.y + my * info.resolution,
  };
}

export function computeFitTransform(
  map: OccupancyGrid,
  canvasWidth: number,
  canvasHeight: number,
  dpr: number,
) {
  const { width, height } = map.info;
  if (!width || !height) {
    return { scale: 1, offsetX: 0, offsetY: 0 };
  }

  const pad = 32 * dpr;
  const sx = (canvasWidth - pad * 2) / width;
  const sy = (canvasHeight - pad * 2) / height;
  let scale = Math.max(0.01, Math.min(sx, sy));
  if (!Number.isFinite(scale)) scale = 1;

  return {
    scale,
    offsetX: (canvasWidth - width * scale) / 2,
    offsetY: (canvasHeight - height * scale) / 2,
  };
}
