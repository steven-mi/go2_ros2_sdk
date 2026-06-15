import {
  forwardRef,
  useCallback,
  useEffect,
  useImperativeHandle,
  useRef,
  useState,
} from 'react';
import { useRos } from '../context/RosContext';
import {
  canvasToMap,
  computeFitTransform,
  mapCacheSignature,
  mapToCanvas,
  rebuildMapCache,
} from '../lib/mapUtils';

export interface MapCanvasHandle {
  zoomIn: () => void;
  zoomOut: () => void;
  fit: () => void;
}

export const MapCanvas = forwardRef<MapCanvasHandle>(function MapCanvas(_props, ref) {
  const { map, mapVersion, robot, goal, mode, publishGoal, publishInitialPose } = useRos();

  const wrapRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const cacheCanvasRef = useRef(document.createElement('canvas'));
  const cacheSigRef = useRef('');

  const [scale, setScale] = useState(1);
  const [offsetX, setOffsetX] = useState(0);
  const [offsetY, setOffsetY] = useState(0);
  const [autoFit, setAutoFit] = useState(true);
  const [panning, setPanning] = useState(false);
  const panStartRef = useRef<{ x: number; y: number } | null>(null);
  const viewStartRef = useRef<{ offsetX: number; offsetY: number } | null>(null);
  const panMovedRef = useRef(false);

  const dpr = window.devicePixelRatio || 1;

  const draw = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    ctx.fillStyle = '#12151c';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    if (!map) {
      ctx.fillStyle = '#8b95a8';
      ctx.font = `${14 * dpr}px sans-serif`;
      ctx.fillText('Waiting for /map…', 24 * dpr, 40 * dpr);
      return;
    }

    const sig = mapCacheSignature(map, mapVersion);
    if (sig !== cacheSigRef.current) {
      rebuildMapCache(map, cacheCanvasRef.current);
      cacheSigRef.current = sig;
    }

    const { info } = map;
    const w = info.width * scale;
    const h = info.height * scale;

    ctx.save();
    ctx.imageSmoothingEnabled = scale < 4;
    ctx.imageSmoothingQuality = 'high';
    ctx.drawImage(cacheCanvasRef.current, offsetX, offsetY, w, h);
    ctx.strokeStyle = 'rgba(79, 140, 255, 0.25)';
    ctx.lineWidth = 1.5 * dpr;
    ctx.strokeRect(offsetX, offsetY, w, h);
    ctx.restore();

    const rp = mapToCanvas(robot.x, robot.y, map, scale, offsetX, offsetY);
    const size = 12 * dpr;
    ctx.save();
    ctx.translate(rp.x, rp.y);
    ctx.rotate(-robot.yaw);
    ctx.shadowColor = 'rgba(79, 140, 255, 0.55)';
    ctx.shadowBlur = 10 * dpr;
    ctx.fillStyle = '#ffffff';
    ctx.beginPath();
    ctx.moveTo(size * 1.1, 0);
    ctx.lineTo(-size * 0.75, size * 0.72);
    ctx.lineTo(-size * 0.75, -size * 0.72);
    ctx.closePath();
    ctx.fill();
    ctx.shadowBlur = 0;
    ctx.fillStyle = '#4f8cff';
    ctx.beginPath();
    ctx.moveTo(size, 0);
    ctx.lineTo(-size * 0.7, size * 0.65);
    ctx.lineTo(-size * 0.7, -size * 0.65);
    ctx.closePath();
    ctx.fill();
    ctx.restore();

    if (goal) {
      const gp = mapToCanvas(goal.x, goal.y, map, scale, offsetX, offsetY);
      const r = 8 * dpr;
      ctx.save();
      ctx.strokeStyle = 'rgba(61, 214, 140, 0.35)';
      ctx.lineWidth = 3 * dpr;
      ctx.beginPath();
      ctx.arc(gp.x, gp.y, r + 4 * dpr, 0, Math.PI * 2);
      ctx.stroke();
      ctx.fillStyle = '#3dd68c';
      ctx.beginPath();
      ctx.arc(gp.x, gp.y, r, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = '#ffffff';
      ctx.beginPath();
      ctx.arc(gp.x, gp.y, r * 0.35, 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
    }
  }, [map, mapVersion, robot, goal, scale, offsetX, offsetY, dpr]);

  const fitMap = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas || !map) return;
    const fit = computeFitTransform(map, canvas.width, canvas.height, dpr);
    setScale(fit.scale);
    setOffsetX(fit.offsetX);
    setOffsetY(fit.offsetY);
    setAutoFit(true);
  }, [map, dpr]);

  const zoomMap = useCallback(
    (factor: number, focalX?: number, focalY?: number) => {
      if (!map || !canvasRef.current) return;
      const fx = focalX ?? canvasRef.current.width / 2;
      const fy = focalY ?? canvasRef.current.height / 2;
      const world = canvasToMap(fx, fy, map, scale, offsetX, offsetY);
      setAutoFit(false);
      const nextScale = Math.max(0.5, Math.min(scale * factor, 80));
      const p = mapToCanvas(world.x, world.y, map, nextScale, offsetX, offsetY);
      setScale(nextScale);
      setOffsetX((prev) => prev + fx - p.x);
      setOffsetY((prev) => prev + fy - p.y);
    },
    [map, scale, offsetX, offsetY],
  );

  useImperativeHandle(
    ref,
    () => ({
      zoomIn: () => zoomMap(1.25),
      zoomOut: () => zoomMap(1 / 1.25),
      fit: fitMap,
    }),
    [zoomMap, fitMap],
  );

  const resizeCanvas = useCallback(() => {
    const wrap = wrapRef.current;
    const canvas = canvasRef.current;
    if (!wrap || !canvas) return;
    const rect = wrap.getBoundingClientRect();
    canvas.width = Math.floor(rect.width * dpr);
    canvas.height = Math.floor(rect.height * dpr);
    if (autoFit) fitMap();
    draw();
  }, [autoFit, dpr, draw, fitMap]);

  useEffect(() => {
    resizeCanvas();
    window.addEventListener('resize', resizeCanvas);
    return () => window.removeEventListener('resize', resizeCanvas);
  }, [resizeCanvas]);

  useEffect(() => {
    if (!map) return;
    if (autoFit) fitMap();
    draw();
  }, [map, mapVersion, robot, goal, scale, offsetX, offsetY, autoFit, draw, fitMap]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const onWheel = (event: WheelEvent) => {
      if (!map || !canvasRef.current) return;
      event.preventDefault();
      const rect = canvasRef.current.getBoundingClientRect();
      const cx = (event.clientX - rect.left) * dpr;
      const cy = (event.clientY - rect.top) * dpr;
      zoomMap(event.deltaY < 0 ? 1.12 : 1 / 1.12, cx, cy);
    };

    canvas.addEventListener('wheel', onWheel, { passive: false });
    return () => canvas.removeEventListener('wheel', onWheel);
  }, [map, dpr, zoomMap]);

  const onPointerDown = (event: React.MouseEvent) => {
    const panGesture = event.button === 1 || (event.button === 0 && event.shiftKey);
    if (!panGesture) return;
    setPanning(true);
    panMovedRef.current = false;
    panStartRef.current = { x: event.clientX, y: event.clientY };
    viewStartRef.current = { offsetX, offsetY };
    event.preventDefault();
  };

  const onPointerMove = (event: React.MouseEvent) => {
    if (!panning || !panStartRef.current || !viewStartRef.current) return;
    const dx = (event.clientX - panStartRef.current.x) * dpr;
    const dy = (event.clientY - panStartRef.current.y) * dpr;
    if (Math.hypot(dx, dy) > 4 * dpr) panMovedRef.current = true;
    setAutoFit(false);
    setOffsetX(viewStartRef.current.offsetX + dx);
    setOffsetY(viewStartRef.current.offsetY + dy);
  };

  const endPan = () => {
    setPanning(false);
    panStartRef.current = null;
    viewStartRef.current = null;
    setTimeout(() => {
      panMovedRef.current = false;
    }, 0);
  };

  const onClick = (event: React.MouseEvent) => {
    if (!map || panning || panMovedRef.current || !canvasRef.current) return;
    const rect = canvasRef.current.getBoundingClientRect();
    const cx = (event.clientX - rect.left) * dpr;
    const cy = (event.clientY - rect.top) * dpr;
    const world = canvasToMap(cx, cy, map, scale, offsetX, offsetY);
    if (mode === 'pose') {
      publishInitialPose(world.x, world.y);
    } else {
      publishGoal(world.x, world.y);
    }
  };

  return (
    <div className="viz-surface map-surface" ref={wrapRef}>
      <canvas
        ref={canvasRef}
        className={panning ? 'map-canvas panning' : 'map-canvas'}
        onClick={onClick}
        onMouseDown={onPointerDown}
        onMouseMove={onPointerMove}
        onMouseUp={endPan}
        onMouseLeave={endPan}
      />
    </div>
  );
});
