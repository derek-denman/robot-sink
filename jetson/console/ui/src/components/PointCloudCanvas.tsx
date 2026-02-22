import { useEffect, useMemo, useRef, useState } from "react";
import type { Point3, PointcloudPayload } from "../types";

type Props = {
  cloud: PointcloudPayload | null;
};

type CameraState = {
  yaw: number;
  pitch: number;
  distance: number;
  targetX: number;
  targetY: number;
  targetZ: number;
};

type DragState = {
  pointerId: number;
  button: number;
  startX: number;
  startY: number;
  origin: CameraState;
};

type Vec3 = [number, number, number];

function normalize(v: Vec3): Vec3 {
  const n = Math.hypot(v[0], v[1], v[2]) || 1;
  return [v[0] / n, v[1] / n, v[2] / n];
}

function cross(a: Vec3, b: Vec3): Vec3 {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function dot(a: Vec3, b: Vec3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function project(
  point: Vec3,
  cameraPos: Vec3,
  right: Vec3,
  up: Vec3,
  forward: Vec3,
  width: number,
  height: number,
  focal: number,
): { x: number; y: number; depth: number } | null {
  const rel: Vec3 = [
    point[0] - cameraPos[0],
    point[1] - cameraPos[1],
    point[2] - cameraPos[2],
  ];
  const depth = dot(rel, forward);
  if (depth <= 0.05) {
    return null;
  }
  const px = dot(rel, right);
  const py = dot(rel, up);
  return {
    x: width * 0.5 + (px / depth) * focal,
    y: height * 0.5 - (py / depth) * focal,
    depth,
  };
}

export function PointCloudCanvas({ cloud }: Props): JSX.Element {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const dragRef = useRef<DragState | null>(null);
  const [camera, setCamera] = useState<CameraState>({
    yaw: 0.7,
    pitch: 0.45,
    distance: 5.5,
    targetX: 0,
    targetY: 0,
    targetZ: 0,
  });

  const points = useMemo(() => {
    if (!cloud?.points?.length) {
      return [] as Point3[];
    }
    return cloud.points;
  }, [cloud]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) {
      return;
    }
    const ctx = canvas.getContext("2d");
    if (!ctx) {
      return;
    }

    const width = canvas.width;
    const height = canvas.height;
    ctx.fillStyle = "#071019";
    ctx.fillRect(0, 0, width, height);

    if (!points.length) {
      ctx.fillStyle = "#8ea2b6";
      ctx.font = "600 14px 'IBM Plex Sans', sans-serif";
      ctx.fillText("Waiting for point cloud topic...", 18, 26);
      return;
    }

    const cp = Math.cos(camera.pitch);
    const sp = Math.sin(camera.pitch);
    const cy = Math.cos(camera.yaw);
    const sy = Math.sin(camera.yaw);
    const cameraPos: Vec3 = [
      camera.targetX + camera.distance * cp * cy,
      camera.targetY + camera.distance * cp * sy,
      camera.targetZ + camera.distance * sp,
    ];
    const target: Vec3 = [camera.targetX, camera.targetY, camera.targetZ];
    const forward = normalize([
      target[0] - cameraPos[0],
      target[1] - cameraPos[1],
      target[2] - cameraPos[2],
    ]);
    let right = normalize(cross(forward, [0, 0, 1]));
    if (!Number.isFinite(right[0])) {
      right = [1, 0, 0];
    }
    const up = normalize(cross(right, forward));
    const focal = Math.min(width, height) * 0.68;

    const drawAxis = (axis: Vec3, color: string) => {
      const p0 = project([0, 0, 0], cameraPos, right, up, forward, width, height, focal);
      const p1 = project(axis, cameraPos, right, up, forward, width, height, focal);
      if (!p0 || !p1) {
        return;
      }
      ctx.beginPath();
      ctx.moveTo(p0.x, p0.y);
      ctx.lineTo(p1.x, p1.y);
      ctx.lineWidth = 2;
      ctx.strokeStyle = color;
      ctx.stroke();
    };

    drawAxis([0.8, 0, 0], "#fd6e5d");
    drawAxis([0, 0.8, 0], "#2fcf8f");
    drawAxis([0, 0, 0.8], "#5aa7ff");

    for (const p of points) {
      const projected = project(
        [Number(p[0]), Number(p[1]), Number(p[2])],
        cameraPos,
        right,
        up,
        forward,
        width,
        height,
        focal,
      );
      if (!projected) {
        continue;
      }
      const alpha = Math.max(0.15, Math.min(0.95, 1.4 / projected.depth));
      const hue = 190 + Math.max(-60, Math.min(60, Number(p[2]) * 28));
      ctx.fillStyle = `hsla(${hue}, 100%, 65%, ${alpha})`;
      ctx.fillRect(projected.x, projected.y, 2, 2);
    }
  }, [camera, points]);

  return (
    <canvas
      ref={canvasRef}
      width={760}
      height={360}
      className="pointcloud-canvas"
      onContextMenu={(event) => event.preventDefault()}
      onPointerDown={(event) => {
        const canvas = canvasRef.current;
        if (!canvas) {
          return;
        }
        canvas.setPointerCapture(event.pointerId);
        dragRef.current = {
          pointerId: event.pointerId,
          button: event.button,
          startX: event.clientX,
          startY: event.clientY,
          origin: { ...camera },
        };
      }}
      onPointerMove={(event) => {
        const drag = dragRef.current;
        if (!drag || drag.pointerId !== event.pointerId) {
          return;
        }
        const dx = event.clientX - drag.startX;
        const dy = event.clientY - drag.startY;

        setCamera((prev) => {
          if (drag.button === 2 || drag.button === 1) {
            const scale = Math.max(0.003, prev.distance * 0.0015);
            return {
              ...prev,
              targetX: drag.origin.targetX - dx * scale,
              targetY: drag.origin.targetY + dy * scale,
            };
          }
          return {
            ...prev,
            yaw: drag.origin.yaw - dx * 0.01,
            pitch: Math.max(-1.35, Math.min(1.35, drag.origin.pitch + dy * 0.01)),
          };
        });
      }}
      onPointerUp={(event) => {
        const canvas = canvasRef.current;
        if (canvas && dragRef.current?.pointerId === event.pointerId) {
          canvas.releasePointerCapture(event.pointerId);
        }
        dragRef.current = null;
      }}
      onPointerCancel={() => {
        dragRef.current = null;
      }}
      onWheel={(event) => {
        event.preventDefault();
        setCamera((prev) => ({
          ...prev,
          distance: Math.max(1.5, Math.min(18, prev.distance * (event.deltaY < 0 ? 0.9 : 1.12))),
        }));
      }}
    />
  );
}
