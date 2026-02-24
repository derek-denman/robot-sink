import { useEffect, useMemo, useRef, useState } from "react";
import type { DetectionPayload, MapOverlayPayload, MapPayload, ScanPoint } from "../types";

type ManualExtrinsics = {
  enabled: boolean;
  x: number;
  y: number;
  yaw: number;
};

type LayerToggles = {
  map: boolean;
  scan: boolean;
  detections: boolean;
  plan: boolean;
};

type Props = {
  map: MapPayload | null;
  overlay: MapOverlayPayload | null;
  detections: DetectionPayload | null;
  width?: number;
  height?: number;
  followPose?: boolean;
  layers?: LayerToggles;
  manualExtrinsics?: ManualExtrinsics;
};

type ViewState = {
  zoom: number;
  panX: number;
  panY: number;
};

function decodeRle(payload: MapPayload | null): Uint8Array | null {
  if (!payload || payload.width <= 0 || payload.height <= 0 || payload.encoding !== "rle_u8") {
    return null;
  }
  const total = payload.width * payload.height;
  const out = new Uint8Array(total);
  let idx = 0;
  for (const run of payload.data_rle || []) {
    if (!Array.isArray(run) || run.length < 2) {
      continue;
    }
    const value = Number(run[0]) & 0xff;
    const count = Math.max(0, Number(run[1]) | 0);
    for (let i = 0; i < count && idx < total; i += 1) {
      out[idx] = value;
      idx += 1;
    }
    if (idx >= total) {
      break;
    }
  }
  return out;
}

function applyManualTransform(points: ScanPoint[], manual: ManualExtrinsics): ScanPoint[] {
  if (!manual.enabled) {
    return points;
  }
  const cosYaw = Math.cos(manual.yaw);
  const sinYaw = Math.sin(manual.yaw);
  return points.map((p) => {
    const x = Number(p[0]);
    const y = Number(p[1]);
    const tx = manual.x + cosYaw * x - sinYaw * y;
    const ty = manual.y + sinYaw * x + cosYaw * y;
    return [tx, ty];
  });
}

function drawPolyline(
  ctx: CanvasRenderingContext2D,
  points: ScanPoint[],
  mapToScreen: (x: number, y: number) => { x: number; y: number },
): void {
  if (!points.length) {
    return;
  }
  ctx.beginPath();
  let started = false;
  for (const point of points) {
    const x = Number(point[0]);
    const y = Number(point[1]);
    if (!Number.isFinite(x) || !Number.isFinite(y)) {
      continue;
    }
    const p = mapToScreen(x, y);
    if (!started) {
      ctx.moveTo(p.x, p.y);
      started = true;
    } else {
      ctx.lineTo(p.x, p.y);
    }
  }
  if (!started) {
    return;
  }
  ctx.strokeStyle = "rgba(103, 235, 143, 0.95)";
  ctx.lineWidth = 2.0;
  ctx.stroke();
}

function drawScanPoints(
  ctx: CanvasRenderingContext2D,
  points: ScanPoint[],
  mapToScreen: (x: number, y: number) => { x: number; y: number },
): void {
  if (!points.length) {
    return;
  }
  ctx.fillStyle = "#3de6ff";
  for (const point of points) {
    const x = Number(point[0]);
    const y = Number(point[1]);
    if (!Number.isFinite(x) || !Number.isFinite(y)) {
      continue;
    }
    const p = mapToScreen(x, y);
    ctx.fillRect(p.x, p.y, 2, 2);
  }
}

function drawRobot(
  ctx: CanvasRenderingContext2D,
  mapToScreen: (x: number, y: number) => { x: number; y: number },
  x: number,
  y: number,
  yaw: number,
): void {
  const c = mapToScreen(x, y);
  const l = 0.38;
  const w = 0.22;
  const tip = mapToScreen(x + Math.cos(yaw) * l, y + Math.sin(yaw) * l);
  const left = mapToScreen(x + Math.cos(yaw + 2.5) * w, y + Math.sin(yaw + 2.5) * w);
  const right = mapToScreen(x + Math.cos(yaw - 2.5) * w, y + Math.sin(yaw - 2.5) * w);

  ctx.beginPath();
  ctx.moveTo(tip.x, tip.y);
  ctx.lineTo(left.x, left.y);
  ctx.lineTo(right.x, right.y);
  ctx.closePath();
  ctx.fillStyle = "#90ffd0";
  ctx.fill();
  ctx.strokeStyle = "rgba(13, 37, 26, 0.9)";
  ctx.lineWidth = 1.2;
  ctx.stroke();

  ctx.beginPath();
  ctx.arc(c.x, c.y, 3.5, 0, Math.PI * 2);
  ctx.fillStyle = "#ffffff";
  ctx.fill();
}

function drawDetectionBoxes(
  ctx: CanvasRenderingContext2D,
  detections: DetectionPayload | null,
  mapToScreen: (x: number, y: number) => { x: number; y: number },
): void {
  if (!detections?.detections?.length) {
    return;
  }
  for (const det of detections.detections) {
    const bev = det.bev;
    if (!bev) {
      continue;
    }
    const cx = Number(bev.x);
    const cy = Number(bev.y);
    const yaw = Number(bev.yaw || 0);
    const length = Math.max(0.06, Number(bev.length || 0.2));
    const width = Math.max(0.06, Number(bev.width || 0.2));

    const hx = length * 0.5;
    const hy = width * 0.5;
    const corners: Array<[number, number]> = [
      [hx, hy],
      [hx, -hy],
      [-hx, -hy],
      [-hx, hy],
    ];
    const cosYaw = Math.cos(yaw);
    const sinYaw = Math.sin(yaw);
    const projected = corners.map(([x, y]) => {
      const px = cx + cosYaw * x - sinYaw * y;
      const py = cy + sinYaw * x + cosYaw * y;
      return mapToScreen(px, py);
    });

    ctx.beginPath();
    ctx.moveTo(projected[0].x, projected[0].y);
    for (let i = 1; i < projected.length; i += 1) {
      ctx.lineTo(projected[i].x, projected[i].y);
    }
    ctx.closePath();
    ctx.strokeStyle = "rgba(255, 84, 206, 0.95)";
    ctx.lineWidth = 1.6;
    ctx.stroke();
    ctx.fillStyle = "rgba(255, 84, 206, 0.15)";
    ctx.fill();

    const label = `${det.class_id || det.id} ${(Number(det.score || 0) * 100).toFixed(0)}%`;
    const lp = mapToScreen(cx, cy);
    ctx.fillStyle = "rgba(255, 125, 222, 0.95)";
    ctx.font = "600 11px 'IBM Plex Sans', sans-serif";
    ctx.fillText(label, lp.x + 4, lp.y - 4);
  }
}

export function FusedViewCanvas({
  map,
  overlay,
  detections,
  width = 900,
  height = 500,
  followPose = false,
  layers = { map: true, scan: true, detections: true, plan: true },
  manualExtrinsics = { enabled: false, x: 0, y: 0, yaw: 0 },
}: Props): JSX.Element {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const mapImageRef = useRef<HTMLCanvasElement | null>(null);
  const [view, setView] = useState<ViewState>({ zoom: 34, panX: 0, panY: 0 });

  const mapBytes = useMemo(() => decodeRle(map), [map]);

  useEffect(() => {
    if (!map || !mapBytes) {
      mapImageRef.current = null;
      return;
    }
    const offscreen = document.createElement("canvas");
    offscreen.width = map.width;
    offscreen.height = map.height;
    const ctx = offscreen.getContext("2d");
    if (!ctx) {
      mapImageRef.current = null;
      return;
    }
    const image = ctx.createImageData(map.width, map.height);
    for (let y = 0; y < map.height; y += 1) {
      for (let x = 0; x < map.width; x += 1) {
        const srcIndex = y * map.width + x;
        const dstY = map.height - 1 - y;
        const dstIndex = (dstY * map.width + x) * 4;
        const cell = mapBytes[srcIndex];
        let shade = 90;
        if (cell === 255) {
          shade = 64;
        } else {
          shade = 255 - Math.round((Math.max(0, Math.min(100, cell)) / 100) * 255);
        }
        image.data[dstIndex] = shade;
        image.data[dstIndex + 1] = shade;
        image.data[dstIndex + 2] = shade;
        image.data[dstIndex + 3] = 255;
      }
    }
    ctx.putImageData(image, 0, 0);
    mapImageRef.current = offscreen;
  }, [map, mapBytes]);

  useEffect(() => {
    if (!followPose || !overlay?.robot_pose) {
      return;
    }
    setView((prev) => ({
      ...prev,
      panX: -Number(overlay.robot_pose?.x || 0) * prev.zoom,
      panY: Number(overlay.robot_pose?.y || 0) * prev.zoom,
    }));
  }, [followPose, overlay?.robot_pose?.x, overlay?.robot_pose?.y]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) {
      return;
    }
    const ctx = canvas.getContext("2d");
    if (!ctx) {
      return;
    }
    const w = canvas.width;
    const h = canvas.height;
    ctx.fillStyle = "#081018";
    ctx.fillRect(0, 0, w, h);

    const mapToScreen = (x: number, y: number): { x: number; y: number } => ({
      x: w * 0.5 + view.panX + x * view.zoom,
      y: h * 0.5 + view.panY - y * view.zoom,
    });

    const spacing = Math.max(16, view.zoom);
    ctx.strokeStyle = "rgba(117, 147, 179, 0.16)";
    ctx.lineWidth = 1;
    for (let x = w * 0.5 + (view.panX % spacing); x <= w; x += spacing) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, h);
      ctx.stroke();
    }
    for (let x = w * 0.5 + (view.panX % spacing); x >= 0; x -= spacing) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, h);
      ctx.stroke();
    }
    for (let y = h * 0.5 + (view.panY % spacing); y <= h; y += spacing) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }
    for (let y = h * 0.5 + (view.panY % spacing); y >= 0; y -= spacing) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }

    if (layers.map && map && mapImageRef.current) {
      const ox = Number(map.origin?.x || 0);
      const oy = Number(map.origin?.y || 0);
      const mapW = Number(map.width || 0) * Number(map.resolution || 0);
      const mapH = Number(map.height || 0) * Number(map.resolution || 0);
      const topLeft = mapToScreen(ox, oy + mapH);
      const bottomRight = mapToScreen(ox + mapW, oy);
      ctx.imageSmoothingEnabled = false;
      ctx.globalAlpha = 0.9;
      ctx.drawImage(mapImageRef.current, topLeft.x, topLeft.y, bottomRight.x - topLeft.x, bottomRight.y - topLeft.y);
      ctx.globalAlpha = 1.0;
    }

    if (layers.plan) {
      drawPolyline(ctx, overlay?.path_points || [], mapToScreen);
    }

    if (layers.scan) {
      const scanPoints =
        overlay?.scan_points && overlay.scan_points.length
          ? overlay.scan_points
          : applyManualTransform((overlay?.scan_points_sensor || []) as ScanPoint[], manualExtrinsics);
      drawScanPoints(ctx, scanPoints as ScanPoint[], mapToScreen);
    }

    if (layers.detections) {
      drawDetectionBoxes(ctx, detections, mapToScreen);
    }

    if (overlay?.robot_pose) {
      drawRobot(
        ctx,
        mapToScreen,
        Number(overlay.robot_pose.x),
        Number(overlay.robot_pose.y),
        Number(overlay.robot_pose.yaw),
      );
    } else {
      drawRobot(ctx, mapToScreen, 0, 0, 0);
    }
  }, [detections, layers, manualExtrinsics, map, overlay, view]);

  return (
    <canvas
      ref={canvasRef}
      width={width}
      height={height}
      className="map-canvas"
      onWheel={(event) => {
        event.preventDefault();
        const delta = event.deltaY < 0 ? 1.08 : 0.92;
        setView((prev) => ({
          ...prev,
          zoom: Math.max(10, Math.min(120, prev.zoom * delta)),
        }));
      }}
    />
  );
}
