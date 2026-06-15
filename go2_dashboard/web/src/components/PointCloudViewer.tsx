import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { Topic } from 'roslib';
import { useRos } from '../context/RosContext';
import { parsePointCloud2 } from '../lib/pointCloudUtils';
import {
  DEFAULT_ORBIT_CAMERA,
  isRobotCentricFrame,
  renderPointCloudCpu,
  type OrbitCamera,
  type RobotOverlay,
} from '../lib/pointCloudRenderer';
import type { PointCloud2Message } from '../lib/rosMessages';

export function PointCloudViewer() {
  const { ros, connected, robot } = useRos();
  const wrapRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const cameraRef = useRef<OrbitCamera>({ ...DEFAULT_ORBIT_CAMERA });
  const draggingRef = useRef(false);
  const dragStartRef = useRef<{ x: number; y: number; yaw: number; pitch: number } | null>(null);
  const rafRef = useRef(0);

  const [cloud, setCloud] = useState<{
    positions: Float32Array;
    colors: Float32Array;
    count: number;
    frameId: string;
  } | null>(null);
  const [topicName, setTopicName] = useState('/pointcloud/filtered');
  const [cameraTick, setCameraTick] = useState(0);
  const gotPrimaryRef = useRef(false);

  const dpr = window.devicePixelRatio || 1;

  const robotOverlay = useMemo<RobotOverlay | null>(() => {
    if (!cloud) return null;
    if (isRobotCentricFrame(cloud.frameId)) {
      return { x: 0, y: 0, z: 0, yaw: 0 };
    }
    return { x: robot.x, y: robot.y, z: 0, yaw: robot.yaw };
  }, [cloud, robot]);

  const draw = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    if (!cloud || !cloud.count) {
      ctx.fillStyle = '#0b0d12';
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      if (robotOverlay) {
        renderPointCloudCpu(
          ctx,
          canvas.width,
          canvas.height,
          dpr,
          new Float32Array(0),
          new Float32Array(0),
          0,
          cameraRef.current,
          robotOverlay,
        );
      } else {
        ctx.fillStyle = '#8b95a8';
        ctx.font = `${14 * dpr}px sans-serif`;
        ctx.fillText('Waiting for point cloud…', 24 * dpr, 40 * dpr);
      }
      return;
    }

    renderPointCloudCpu(
      ctx,
      canvas.width,
      canvas.height,
      dpr,
      cloud.positions,
      cloud.colors,
      cloud.count,
      cameraRef.current,
      robotOverlay,
    );
  }, [cloud, dpr, cameraTick, robotOverlay]);

  const resizeCanvas = useCallback(() => {
    const wrap = wrapRef.current;
    const canvas = canvasRef.current;
    if (!wrap || !canvas) return;
    const rect = wrap.getBoundingClientRect();
    canvas.width = Math.max(1, Math.floor(rect.width * dpr));
    canvas.height = Math.max(1, Math.floor(rect.height * dpr));
    draw();
  }, [dpr, draw]);

  useEffect(() => {
    resizeCanvas();
    window.addEventListener('resize', resizeCanvas);
    return () => window.removeEventListener('resize', resizeCanvas);
  }, [resizeCanvas]);

  useEffect(() => {
    draw();
  }, [draw]);

  useEffect(() => {
    if (!ros || !connected) {
      setCloud(null);
      return;
    }

    gotPrimaryRef.current = false;

    const topicOptions = {
      ros,
      messageType: 'sensor_msgs/msg/PointCloud2',
      throttle_rate: 200,
      compression: 'cbor' as const,
    };

    const primary = new Topic({
      ...topicOptions,
      name: '/pointcloud/filtered',
    });

    const fallbackAggregated = new Topic({
      ...topicOptions,
      name: '/pointcloud/aggregated',
    });

    const fallbackRaw = new Topic({
      ...topicOptions,
      name: '/point_cloud2',
    });

    primary.subscribe((msg) => {
      gotPrimaryRef.current = true;
      const parsed = parsePointCloud2(msg as PointCloud2Message);
      if (parsed) {
        setCloud(parsed);
        setTopicName('/pointcloud/filtered');
      }
    });

    const fallbackTimer = window.setTimeout(() => {
      if (gotPrimaryRef.current) return;
      fallbackAggregated.subscribe((msg) => {
        const parsed = parsePointCloud2(msg as PointCloud2Message);
        if (parsed) {
          setCloud(parsed);
          setTopicName('/pointcloud/aggregated');
        }
      });
    }, 2000);

    const rawFallbackTimer = window.setTimeout(() => {
      if (gotPrimaryRef.current) return;
      fallbackRaw.subscribe((msg) => {
        const parsed = parsePointCloud2(msg as PointCloud2Message);
        if (parsed) {
          setCloud(parsed);
          setTopicName('/point_cloud2');
        }
      });
    }, 4000);

    return () => {
      window.clearTimeout(fallbackTimer);
      window.clearTimeout(rawFallbackTimer);
      primary.unsubscribe();
      fallbackAggregated.unsubscribe();
      fallbackRaw.unsubscribe();
    };
  }, [ros, connected]);

  const scheduleDraw = useCallback(() => {
    cancelAnimationFrame(rafRef.current);
    rafRef.current = requestAnimationFrame(() => setCameraTick((t) => t + 1));
  }, []);

  const onPointerDown = (e: React.PointerEvent<HTMLCanvasElement>) => {
    draggingRef.current = true;
    dragStartRef.current = {
      x: e.clientX,
      y: e.clientY,
      yaw: cameraRef.current.yaw,
      pitch: cameraRef.current.pitch,
    };
    e.currentTarget.setPointerCapture(e.pointerId);
  };

  const onPointerMove = (e: React.PointerEvent<HTMLCanvasElement>) => {
    if (!draggingRef.current || !dragStartRef.current) return;
    const dx = e.clientX - dragStartRef.current.x;
    const dy = e.clientY - dragStartRef.current.y;
    cameraRef.current.yaw = dragStartRef.current.yaw + dx * 0.008;
    cameraRef.current.pitch = Math.max(
      -1.2,
      Math.min(1.2, dragStartRef.current.pitch + dy * 0.008),
    );
    scheduleDraw();
  };

  const onPointerUp = (e: React.PointerEvent<HTMLCanvasElement>) => {
    draggingRef.current = false;
    dragStartRef.current = null;
    e.currentTarget.releasePointerCapture(e.pointerId);
  };

  const onWheel = (e: React.WheelEvent<HTMLCanvasElement>) => {
    e.preventDefault();
    const factor = e.deltaY > 0 ? 0.92 : 1.08;
    cameraRef.current.zoom = Math.max(0.35, Math.min(4, cameraRef.current.zoom * factor));
    scheduleDraw();
  };

  useEffect(() => () => cancelAnimationFrame(rafRef.current), []);

  return (
    <div className="viz-surface pointcloud-surface" ref={wrapRef}>
      <canvas
        ref={canvasRef}
        className="pointcloud-canvas"
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerUp}
        onPointerLeave={onPointerUp}
        onWheel={onWheel}
      />
      <div className="pointcloud-overlay">
        {cloud ? (
          <span>
            {cloud.count.toLocaleString()} points · {cloud.frameId} · {topicName} · CPU render
          </span>
        ) : (
          <span>Waiting for point cloud…</span>
        )}
      </div>
      <div className="pointcloud-hint">Drag to orbit · scroll to zoom</div>
    </div>
  );
}
