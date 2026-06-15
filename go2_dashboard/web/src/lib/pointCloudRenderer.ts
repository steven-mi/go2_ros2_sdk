export interface OrbitCamera {
  yaw: number;
  pitch: number;
  distance: number;
  zoom: number;
}

export const DEFAULT_ORBIT_CAMERA: OrbitCamera = {
  yaw: -Math.PI / 4,
  pitch: 0.55,
  distance: 6,
  zoom: 1,
};

export interface RobotOverlay {
  x: number;
  y: number;
  z: number;
  yaw: number;
}

/** True when point cloud points are expressed in the robot body frame. */
export function isRobotCentricFrame(frameId: string): boolean {
  const frame = frameId.replace(/^\//, '');
  return frame === 'base_link' || frame.endsWith('/base_link');
}

function projectPoint(
  x: number,
  y: number,
  z: number,
  camera: OrbitCamera,
  width: number,
  height: number,
): { sx: number; sy: number; depth: number } | null {
  const cosY = Math.cos(camera.yaw);
  const sinY = Math.sin(camera.yaw);
  const cosP = Math.cos(camera.pitch);
  const sinP = Math.sin(camera.pitch);

  const rx = x * cosY - y * sinY;
  const ry = x * sinY + y * cosY;
  const rz = z;

  const cy = ry * cosP - rz * sinP;
  const cz = ry * sinP + rz * cosP;

  const depth = cy + camera.distance;
  if (depth <= 0.15) return null;

  const fov = (55 * Math.PI) / 180;
  const focal = (height * 0.5) / Math.tan(fov * 0.5);
  const scale = focal * camera.zoom;

  const sx = (rx / depth) * scale + width * 0.5;
  const sy = (-cz / depth) * scale + height * 0.5;

  if (sx < -2 || sy < -2 || sx > width + 2 || sy > height + 2) return null;

  return { sx, sy, depth };
}

function strokeSegment(
  ctx: CanvasRenderingContext2D,
  x1: number,
  y1: number,
  z1: number,
  x2: number,
  y2: number,
  z2: number,
  camera: OrbitCamera,
  width: number,
  height: number,
) {
  const p1 = projectPoint(x1, y1, z1, camera, width, height);
  const p2 = projectPoint(x2, y2, z2, camera, width, height);
  if (!p1 || !p2) return;
  ctx.moveTo(p1.sx, p1.sy);
  ctx.lineTo(p2.sx, p2.sy);
}

function drawGrid(
  ctx: CanvasRenderingContext2D,
  camera: OrbitCamera,
  width: number,
  height: number,
  dpr: number,
) {
  const extent = 15;
  const step = 1;
  const majorEvery = 5;
  const cosY = Math.cos(camera.yaw);
  const sinY = Math.sin(camera.yaw);

  for (let i = -extent; i <= extent; i += step) {
    const major = i % majorEvery === 0;
    ctx.strokeStyle = major ? 'rgba(42, 49, 66, 0.9)' : 'rgba(26, 31, 43, 0.75)';
    ctx.lineWidth = (major ? 1.2 : 0.8) * dpr;
    ctx.beginPath();
    strokeSegment(ctx, i, -extent, 0, i, extent, 0, camera, width, height);
    ctx.stroke();
    ctx.beginPath();
    strokeSegment(ctx, -extent, i, 0, extent, i, 0, camera, width, height);
    ctx.stroke();
  }

  const origin = projectPoint(0, 0, 0, camera, width, height);
  const xAxis = projectPoint(cosY * 1.2, sinY * 1.2, 0, camera, width, height);
  const zAxis = projectPoint(0, 0, 1.2, camera, width, height);

  ctx.strokeStyle = 'rgba(79, 140, 255, 0.35)';
  ctx.lineWidth = 1.5 * dpr;
  ctx.beginPath();
  if (origin && xAxis) {
    ctx.moveTo(origin.sx, origin.sy);
    ctx.lineTo(xAxis.sx, xAxis.sy);
  }
  if (origin && zAxis) {
    ctx.moveTo(origin.sx, origin.sy);
    ctx.lineTo(zAxis.sx, zAxis.sy);
  }
  ctx.stroke();

  if (origin && xAxis) {
    const size = 10 * dpr;
    const angle = Math.atan2(xAxis.sy - origin.sy, xAxis.sx - origin.sx);
    ctx.save();
    ctx.translate(origin.sx, origin.sy);
    ctx.rotate(angle);
    ctx.fillStyle = '#4f8cff';
    ctx.beginPath();
    ctx.moveTo(size, 0);
    ctx.lineTo(-size * 0.65, size * 0.55);
    ctx.lineTo(-size * 0.65, -size * 0.55);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }
}

type WorldPoint = { x: number; y: number; z: number };

function localToWorld(
  lx: number,
  ly: number,
  lz: number,
  robot: RobotOverlay,
): WorldPoint {
  const cos = Math.cos(robot.yaw);
  const sin = Math.sin(robot.yaw);
  return {
    x: robot.x + lx * cos - ly * sin,
    y: robot.y + lx * sin + ly * cos,
    z: robot.z + lz,
  };
}

function projectPolygon(
  ctx: CanvasRenderingContext2D,
  points: WorldPoint[],
  camera: OrbitCamera,
  width: number,
  height: number,
): boolean {
  const projected = points
    .map((p) => projectPoint(p.x, p.y, p.z, camera, width, height))
    .filter((p): p is { sx: number; sy: number; depth: number } => p !== null);
  if (projected.length < 3) return false;

  ctx.beginPath();
  ctx.moveTo(projected[0].sx, projected[0].sy);
  for (let i = 1; i < projected.length; i += 1) {
    ctx.lineTo(projected[i].sx, projected[i].sy);
  }
  ctx.closePath();
  return true;
}

function drawGo2Overlay(
  ctx: CanvasRenderingContext2D,
  camera: OrbitCamera,
  width: number,
  height: number,
  dpr: number,
  robot: RobotOverlay,
) {
  const bodyHalfL = 0.34;
  const bodyHalfW = 0.14;
  const bodyZ = 0.17;
  const legZ = 0.05;
  const legHalf = 0.07;

  const bodyCorners: WorldPoint[] = [
    localToWorld(bodyHalfL, bodyHalfW, bodyZ, robot),
    localToWorld(bodyHalfL, -bodyHalfW, bodyZ, robot),
    localToWorld(-bodyHalfL, -bodyHalfW, bodyZ, robot),
    localToWorld(-bodyHalfL, bodyHalfW, bodyZ, robot),
  ];

  const legs: WorldPoint[][] = [
    [
      localToWorld(0.24, 0.13, legZ, robot),
      localToWorld(0.24 + legHalf, 0.13 + legHalf, legZ, robot),
      localToWorld(0.24 + legHalf, 0.13 - legHalf, legZ, robot),
      localToWorld(0.24, 0.13 - legHalf, legZ, robot),
    ],
    [
      localToWorld(0.24, -0.13, legZ, robot),
      localToWorld(0.24 + legHalf, -0.13 + legHalf, legZ, robot),
      localToWorld(0.24 + legHalf, -0.13 - legHalf, legZ, robot),
      localToWorld(0.24, -0.13 - legHalf, legZ, robot),
    ],
    [
      localToWorld(-0.24, 0.13, legZ, robot),
      localToWorld(-0.24 - legHalf, 0.13 + legHalf, legZ, robot),
      localToWorld(-0.24 - legHalf, 0.13 - legHalf, legZ, robot),
      localToWorld(-0.24, 0.13 - legHalf, legZ, robot),
    ],
    [
      localToWorld(-0.24, -0.13, legZ, robot),
      localToWorld(-0.24 - legHalf, -0.13 + legHalf, legZ, robot),
      localToWorld(-0.24 - legHalf, -0.13 - legHalf, legZ, robot),
      localToWorld(-0.24, -0.13 - legHalf, legZ, robot),
    ],
  ];

  const nose = localToWorld(bodyHalfL + 0.08, 0, bodyZ, robot);
  const noseLeft = localToWorld(bodyHalfL, 0.06, bodyZ, robot);
  const noseRight = localToWorld(bodyHalfL, -0.06, bodyZ, robot);

  ctx.save();
  ctx.lineJoin = 'round';

  for (const leg of legs) {
    if (!projectPolygon(ctx, leg, camera, width, height)) continue;
    ctx.fillStyle = 'rgba(200, 210, 230, 0.92)';
    ctx.strokeStyle = 'rgba(79, 140, 255, 0.55)';
    ctx.lineWidth = 1.2 * dpr;
    ctx.fill();
    ctx.stroke();
  }

  if (projectPolygon(ctx, bodyCorners, camera, width, height)) {
    ctx.fillStyle = 'rgba(79, 140, 255, 0.82)';
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.9)';
    ctx.lineWidth = 1.5 * dpr;
    ctx.fill();
    ctx.stroke();
  }

  if (projectPolygon(ctx, [nose, noseLeft, noseRight], camera, width, height)) {
    ctx.fillStyle = 'rgba(255, 255, 255, 0.95)';
    ctx.fill();
  }

  const heading = projectPoint(nose.x, nose.y, nose.z, camera, width, height);
  const bodyFront = projectPoint(
    bodyCorners[0].x,
    bodyCorners[0].y,
    bodyCorners[0].z,
    camera,
    width,
    height,
  );
  if (heading && bodyFront) {
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.75)';
    ctx.lineWidth = 1.2 * dpr;
    ctx.beginPath();
    ctx.moveTo(bodyFront.sx, bodyFront.sy);
    ctx.lineTo(heading.sx, heading.sy);
    ctx.stroke();
  }

  ctx.restore();
}

export function renderPointCloudCpu(
  ctx: CanvasRenderingContext2D,
  width: number,
  height: number,
  dpr: number,
  positions: Float32Array,
  colors: Float32Array,
  count: number,
  camera: OrbitCamera,
  robot?: RobotOverlay | null,
) {
  ctx.fillStyle = '#0b0d12';
  ctx.fillRect(0, 0, width, height);

  drawGrid(ctx, camera, width, height, dpr);

  if (count) {
    const pixelCount = width * height;
    const depthBuf = new Float32Array(pixelCount);
    depthBuf.fill(Number.POSITIVE_INFINITY);

    const buffer = new ArrayBuffer(pixelCount * 4);
    const pixels32 = new Uint32Array(buffer);
    pixels32.fill(0xff0d0d0b); // #0b0d12 + alpha

    const drawStride = count > 60_000 ? 2 : 1;
    const pointRadius = drawStride > 1 ? 1 : 0;

    for (let i = 0; i < count; i += drawStride) {
      const idx = i * 3;
      const p = projectPoint(
        positions[idx],
        positions[idx + 1],
        positions[idx + 2],
        camera,
        width,
        height,
      );
      if (!p) continue;

      const ri = (colors[idx] * 255) | 0;
      const gi = (colors[idx + 1] * 255) | 0;
      const bi = (colors[idx + 2] * 255) | 0;

      const r = pointRadius;
      const x0 = Math.max(0, (p.sx | 0) - r);
      const y0 = Math.max(0, (p.sy | 0) - r);
      const x1 = Math.min(width - 1, (p.sx | 0) + r);
      const y1 = Math.min(height - 1, (p.sy | 0) + r);

      for (let py = y0; py <= y1; py += 1) {
        for (let px = x0; px <= x1; px += 1) {
          const pi = py * width + px;
          if (p.depth < depthBuf[pi]) {
            depthBuf[pi] = p.depth;
            pixels32[pi] = (255 << 24) | (bi << 16) | (gi << 8) | ri;
          }
        }
      }
    }

    const image = ctx.createImageData(width, height);
    image.data.set(new Uint8ClampedArray(buffer));
    ctx.putImageData(image, 0, 0);
  }

  if (robot) {
    drawGo2Overlay(ctx, camera, width, height, dpr, robot);
  }
}
