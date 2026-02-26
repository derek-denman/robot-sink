import { useEffect, useMemo, useRef, useState } from "react";
import type {
  DetectionPayload,
  MapOverlayPayload,
  MapPayload,
  ObstacleMapPayload,
  ScanPoint,
} from "../types";

type ManualExtrinsics = {
  enabled: boolean;
  x: number;
  y: number;
  yaw: number;
};

type LayerToggles = {
  map: boolean;
  obstacle: boolean;
  scan: boolean;
  detections: boolean;
  plan: boolean;
  drivable: boolean;
};

type TfChainOverlay = {
  fixed_frame: string;
  selected_fixed_frame: string;
  scan_to_fixed_ok: boolean;
  detections_to_fixed_ok: boolean;
  has_odom_base_tf: boolean;
  has_base_laser_tf: boolean;
  has_base_oak_tf: boolean;
  missing_pairs: string[];
};

type RenderStats = {
  fps: number;
  droppedFrames: number;
  frameTimeMs: number;
};

type Props = {
  map: MapPayload | null;
  obstacleMap?: ObstacleMapPayload | null;
  overlay: MapOverlayPayload | null;
  detections: DetectionPayload | null;
  width?: number;
  height?: number;
  followPose?: boolean;
  panEnabled?: boolean;
  recenterSignal?: number;
  layers?: LayerToggles;
  manualExtrinsics?: ManualExtrinsics;
  showTfOverlay?: boolean;
  showAxesOverlay?: boolean;
  tfChain?: TfChainOverlay;
  onPanStart?: () => void;
  onRenderStats?: (stats: RenderStats) => void;
};

type ViewState = {
  zoom: number;
  panX: number;
  panY: number;
};

type DragState = {
  pointerId: number;
  startX: number;
  startY: number;
  originPanX: number;
  originPanY: number;
};

type DetectionTrack = {
  id: string;
  class_id: string;
  score: number;
  x: number;
  y: number;
  yaw: number;
  length: number;
  width: number;
  lastSeenMs: number;
};

function decodeRleU8(
  width: number,
  height: number,
  encoding: string,
  dataRle: Array<[number, number]>,
): Uint8Array | null {
  if (width <= 0 || height <= 0 || encoding !== "rle_u8") {
    return null;
  }
  const total = width * height;
  const out = new Uint8Array(total);
  let idx = 0;
  for (const run of dataRle || []) {
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
  ctx.strokeStyle = "rgba(62, 244, 171, 0.96)";
  ctx.shadowColor = "rgba(63, 243, 170, 0.35)";
  ctx.shadowBlur = 5;
  ctx.lineWidth = 2.2;
  ctx.stroke();
  ctx.shadowBlur = 0;
}

function drawScanPoints(
  ctx: CanvasRenderingContext2D,
  points: ScanPoint[],
  mapToScreen: (x: number, y: number) => { x: number; y: number },
): void {
  if (!points.length) {
    return;
  }
  ctx.fillStyle = "rgba(87, 231, 255, 0.95)";
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
  ctx.fillStyle = "#aab3bf";
  ctx.fill();
  ctx.strokeStyle = "rgba(228, 238, 255, 0.85)";
  ctx.lineWidth = 1.4;
  ctx.stroke();

  ctx.beginPath();
  ctx.arc(c.x, c.y, 3.5, 0, Math.PI * 2);
  ctx.fillStyle = "#ffffff";
  ctx.fill();
}

function drawDetectionBoxes(
  ctx: CanvasRenderingContext2D,
  tracks: DetectionTrack[],
  mapToScreen: (x: number, y: number) => { x: number; y: number },
  nowMs: number,
): void {
  if (!tracks.length) {
    return;
  }
  for (const det of tracks) {
    const ageSec = Math.max(0, (nowMs - det.lastSeenMs) / 1000);
    const alpha = Math.max(0, Math.min(1, 1 - ageSec / 1.5));
    if (alpha <= 0.01) {
      continue;
    }

    const cx = det.x;
    const cy = det.y;
    const yaw = det.yaw;
    const length = Math.max(0.06, det.length);
    const width = Math.max(0.06, det.width);

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
    ctx.strokeStyle = `rgba(255, 84, 206, ${0.95 * alpha})`;
    ctx.lineWidth = 1.8;
    ctx.stroke();
    ctx.fillStyle = `rgba(255, 84, 206, ${0.16 * alpha})`;
    ctx.fill();

    const label = `${det.class_id || det.id} ${(det.score * 100).toFixed(0)}%`;
    const lp = mapToScreen(cx, cy);
    ctx.fillStyle = `rgba(255, 170, 236, ${0.95 * alpha})`;
    ctx.font = "600 11px 'IBM Plex Sans', sans-serif";
    ctx.fillText(label, lp.x + 4, lp.y - 4);
  }
}

function drawDrivableWedge(
  ctx: CanvasRenderingContext2D,
  mapToScreen: (x: number, y: number) => { x: number; y: number },
  x: number,
  y: number,
  yaw: number,
): void {
  const radius = 4.2;
  const halfFov = 0.62;
  const steps = 28;

  ctx.beginPath();
  const origin = mapToScreen(x, y);
  ctx.moveTo(origin.x, origin.y);
  for (let i = 0; i <= steps; i += 1) {
    const t = -halfFov + (i / steps) * halfFov * 2;
    const px = x + Math.cos(yaw + t) * radius;
    const py = y + Math.sin(yaw + t) * radius;
    const pp = mapToScreen(px, py);
    ctx.lineTo(pp.x, pp.y);
  }
  ctx.closePath();
  ctx.fillStyle = "rgba(255, 135, 45, 0.24)";
  ctx.fill();
}

function drawAxesOverlay(
  ctx: CanvasRenderingContext2D,
  height: number,
  frameLabel: string,
): void {
  const ox = 58;
  const oy = height - 42;
  const axis = 30;

  ctx.fillStyle = "rgba(8, 17, 30, 0.8)";
  ctx.fillRect(12, height - 84, 140, 72);

  ctx.strokeStyle = "#fd6e5d";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(ox, oy);
  ctx.lineTo(ox + axis, oy);
  ctx.stroke();
  ctx.fillStyle = "#fd6e5d";
  ctx.font = "600 10px 'IBM Plex Sans', sans-serif";
  ctx.fillText("X", ox + axis + 5, oy + 4);

  ctx.strokeStyle = "#44de98";
  ctx.beginPath();
  ctx.moveTo(ox, oy);
  ctx.lineTo(ox, oy - axis);
  ctx.stroke();
  ctx.fillStyle = "#44de98";
  ctx.fillText("Y", ox - 12, oy - axis - 2);

  ctx.beginPath();
  ctx.arc(ox, oy, 4, 0, Math.PI * 2);
  ctx.fillStyle = "#79b8ff";
  ctx.fill();
  ctx.fillStyle = "#d8e6ff";
  ctx.fillText(`frame: ${frameLabel || "n/a"}`, 16, height - 16);
}

function drawTfOverlay(
  ctx: CanvasRenderingContext2D,
  width: number,
  tfChain: TfChainOverlay,
): void {
  const lines = [
    `Fixed: ${tfChain.fixed_frame} (selected ${tfChain.selected_fixed_frame})`,
    `scan->fixed: ${tfChain.scan_to_fixed_ok ? "OK" : "MISSING"}`,
    `det->fixed: ${tfChain.detections_to_fixed_ok ? "OK" : "MISSING"}`,
    `odom->base_link: ${tfChain.has_odom_base_tf ? "OK" : "MISSING"}`,
    `base_link->laser: ${tfChain.has_base_laser_tf ? "OK" : "MISSING"}`,
    `base_link->oak: ${tfChain.has_base_oak_tf ? "OK" : "MISSING"}`,
  ];
  if (tfChain.missing_pairs.length) {
    lines.push(`missing: ${tfChain.missing_pairs.join(", ")}`);
  }

  const boxWidth = Math.min(520, width - 24);
  const boxHeight = 18 + lines.length * 14;
  ctx.fillStyle = "rgba(7, 14, 24, 0.85)";
  ctx.fillRect(12, 12, boxWidth, boxHeight);
  ctx.strokeStyle = "rgba(95, 137, 182, 0.5)";
  ctx.strokeRect(12, 12, boxWidth, boxHeight);

  ctx.fillStyle = "#d3e6ff";
  ctx.font = "600 11px 'IBM Plex Sans', sans-serif";
  for (let i = 0; i < lines.length; i += 1) {
    ctx.fillText(lines[i], 18, 28 + i * 14);
  }
}

function buildMapTexture(map: MapPayload | null, mapBytes: Uint8Array | null): HTMLCanvasElement | null {
  if (!map || !mapBytes) {
    return null;
  }
  const offscreen = document.createElement("canvas");
  offscreen.width = map.width;
  offscreen.height = map.height;
  const ctx = offscreen.getContext("2d");
  if (!ctx) {
    return null;
  }
  const image = ctx.createImageData(map.width, map.height);
  for (let y = 0; y < map.height; y += 1) {
    for (let x = 0; x < map.width; x += 1) {
      const srcIndex = y * map.width + x;
      const dstY = map.height - 1 - y;
      const dstIndex = (dstY * map.width + x) * 4;
      const cell = mapBytes[srcIndex];
      let shade = 84;
      if (cell === 255) {
        shade = 52;
      } else {
        shade = 255 - Math.round((Math.max(0, Math.min(100, cell)) / 100) * 255);
      }
      image.data[dstIndex] = shade;
      image.data[dstIndex + 1] = shade;
      image.data[dstIndex + 2] = shade;
      image.data[dstIndex + 3] = 235;
    }
  }
  ctx.putImageData(image, 0, 0);
  return offscreen;
}

function buildObstacleTexture(
  obstacleMap: ObstacleMapPayload | null | undefined,
  obstacleBytes: Uint8Array | null,
): HTMLCanvasElement | null {
  if (!obstacleMap || !obstacleBytes) {
    return null;
  }
  const offscreen = document.createElement("canvas");
  offscreen.width = obstacleMap.width;
  offscreen.height = obstacleMap.height;
  const ctx = offscreen.getContext("2d");
  if (!ctx) {
    return null;
  }
  const image = ctx.createImageData(obstacleMap.width, obstacleMap.height);
  const free = Number(obstacleMap.value_mapping?.free ?? 0);
  const occupied = Number(obstacleMap.value_mapping?.occupied ?? 253);
  const unknown = Number(obstacleMap.value_mapping?.unknown ?? 255);
  const inflationMin = Number(obstacleMap.value_mapping?.inflation_min ?? 1);
  const inflationMax = Number(obstacleMap.value_mapping?.inflation_max ?? 252);
  const span = Math.max(1, inflationMax - inflationMin);

  for (let y = 0; y < obstacleMap.height; y += 1) {
    for (let x = 0; x < obstacleMap.width; x += 1) {
      const srcIndex = y * obstacleMap.width + x;
      const dstY = obstacleMap.height - 1 - y;
      const dstIndex = (dstY * obstacleMap.width + x) * 4;
      const cell = obstacleBytes[srcIndex];

      if (cell === unknown) {
        image.data[dstIndex + 3] = 0;
        continue;
      }

      if (cell <= free) {
        image.data[dstIndex + 3] = 0;
        continue;
      }

      if (cell >= occupied) {
        image.data[dstIndex] = 255;
        image.data[dstIndex + 1] = 120;
        image.data[dstIndex + 2] = 40;
        image.data[dstIndex + 3] = 210;
        continue;
      }

      const norm = Math.max(0, Math.min(1, (cell - inflationMin) / span));
      image.data[dstIndex] = 255;
      image.data[dstIndex + 1] = 138;
      image.data[dstIndex + 2] = 62;
      image.data[dstIndex + 3] = Math.round(35 + norm * 130);
    }
  }

  ctx.putImageData(image, 0, 0);
  return offscreen;
}

export function FusedViewCanvas({
  map,
  obstacleMap = null,
  overlay,
  detections,
  width = 900,
  height = 500,
  followPose = false,
  panEnabled = false,
  recenterSignal = 0,
  layers = {
    map: true,
    obstacle: true,
    scan: true,
    detections: true,
    plan: true,
    drivable: true,
  },
  manualExtrinsics = { enabled: false, x: 0, y: 0, yaw: 0 },
  showTfOverlay = false,
  showAxesOverlay = false,
  tfChain,
  onPanStart,
  onRenderStats,
}: Props): JSX.Element {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const dragRef = useRef<DragState | null>(null);
  const mapImageRef = useRef<HTMLCanvasElement | null>(null);
  const obstacleImageRef = useRef<HTMLCanvasElement | null>(null);
  const detectionTracksRef = useRef<Record<string, DetectionTrack>>({});
  const recenterRef = useRef(recenterSignal);
  const statsRef = useRef({
    lastRenderMs: 0,
    lastReportMs: 0,
    frameCount: 0,
    droppedFrames: 0,
  });
  const [view, setView] = useState<ViewState>({ zoom: 34, panX: 0, panY: 0 });

  const mapBytes = useMemo(
    () => decodeRleU8(Number(map?.width || 0), Number(map?.height || 0), String(map?.encoding || ""), map?.data_rle || []),
    [map],
  );
  const obstacleBytes = useMemo(
    () =>
      decodeRleU8(
        Number(obstacleMap?.width || 0),
        Number(obstacleMap?.height || 0),
        String(obstacleMap?.encoding || ""),
        obstacleMap?.data_rle || [],
      ),
    [obstacleMap],
  );

  useEffect(() => {
    mapImageRef.current = buildMapTexture(map, mapBytes);
  }, [map, mapBytes]);

  useEffect(() => {
    obstacleImageRef.current = buildObstacleTexture(obstacleMap, obstacleBytes);
  }, [obstacleMap, obstacleBytes]);

  useEffect(() => {
    let shouldCenter = followPose;
    if (recenterSignal !== recenterRef.current) {
      recenterRef.current = recenterSignal;
      shouldCenter = true;
    }
    if (!shouldCenter) {
      return;
    }

    const poseX = Number(overlay?.robot_pose?.x || 0);
    const poseY = Number(overlay?.robot_pose?.y || 0);
    setView((prev) => ({
      ...prev,
      panX: -poseX * prev.zoom,
      panY: poseY * prev.zoom,
    }));
  }, [followPose, overlay?.robot_pose?.x, overlay?.robot_pose?.y, recenterSignal]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) {
      return;
    }
    const ctx = canvas.getContext("2d");
    if (!ctx) {
      return;
    }

    const nowMs = performance.now();
    const lastRenderMs = statsRef.current.lastRenderMs;
    const frameTimeMs = lastRenderMs > 0 ? nowMs - lastRenderMs : 0;
    if (frameTimeMs > 80) {
      statsRef.current.droppedFrames += 1;
    }
    statsRef.current.lastRenderMs = nowMs;
    statsRef.current.frameCount += 1;

    if (onRenderStats && nowMs - statsRef.current.lastReportMs > 500) {
      const spanSec = Math.max(0.001, (nowMs - statsRef.current.lastReportMs) / 1000);
      const fps = statsRef.current.frameCount / spanSec;
      onRenderStats({
        fps: Number(fps.toFixed(2)),
        droppedFrames: statsRef.current.droppedFrames,
        frameTimeMs: Number(frameTimeMs.toFixed(2)),
      });
      statsRef.current.lastReportMs = nowMs;
      statsRef.current.frameCount = 0;
      statsRef.current.droppedFrames = 0;
    }

    const w = canvas.width;
    const h = canvas.height;
    ctx.fillStyle = "#06111e";
    ctx.fillRect(0, 0, w, h);

    const mapToScreen = (x: number, y: number): { x: number; y: number } => ({
      x: w * 0.5 + view.panX + x * view.zoom,
      y: h * 0.5 + view.panY - y * view.zoom,
    });

    const spacing = Math.max(16, view.zoom);
    ctx.strokeStyle = "rgba(83, 133, 181, 0.2)";
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
      ctx.globalAlpha = 0.86;
      ctx.drawImage(mapImageRef.current, topLeft.x, topLeft.y, bottomRight.x - topLeft.x, bottomRight.y - topLeft.y);
      ctx.globalAlpha = 1.0;
    }

    if (layers.obstacle && obstacleMap && obstacleImageRef.current) {
      const ox = Number(obstacleMap.origin?.x || 0);
      const oy = Number(obstacleMap.origin?.y || 0);
      const obsW = Number(obstacleMap.width || 0) * Number(obstacleMap.resolution || 0);
      const obsH = Number(obstacleMap.height || 0) * Number(obstacleMap.resolution || 0);
      const topLeft = mapToScreen(ox, oy + obsH);
      const bottomRight = mapToScreen(ox + obsW, oy);
      ctx.imageSmoothingEnabled = false;
      ctx.globalAlpha = 0.94;
      ctx.drawImage(
        obstacleImageRef.current,
        topLeft.x,
        topLeft.y,
        bottomRight.x - topLeft.x,
        bottomRight.y - topLeft.y,
      );
      ctx.globalAlpha = 1.0;
    }

    if (layers.drivable) {
      const px = Number(overlay?.robot_pose?.x || 0);
      const py = Number(overlay?.robot_pose?.y || 0);
      const pyaw = Number(overlay?.robot_pose?.yaw || 0);
      drawDrivableWedge(ctx, mapToScreen, px, py, pyaw);
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
      const now = performance.now();
      const tracks = detectionTracksRef.current;
      const activeKeys = new Set<string>();
      for (const det of detections?.detections || []) {
        const bev = det.bev;
        if (!bev) {
          continue;
        }
        const key = `${det.id || det.class_id || "det"}:${det.class_id || "class"}`;
        activeKeys.add(key);
        const prev = tracks[key];
        const nextX = Number(bev.x || 0);
        const nextY = Number(bev.y || 0);
        const nextYaw = Number(bev.yaw || 0);
        const nextLength = Number(bev.length || 0.2);
        const nextWidth = Number(bev.width || 0.2);
        if (!prev) {
          tracks[key] = {
            id: det.id || key,
            class_id: det.class_id || det.id || "obj",
            score: Number(det.score || 0),
            x: nextX,
            y: nextY,
            yaw: nextYaw,
            length: nextLength,
            width: nextWidth,
            lastSeenMs: now,
          };
          continue;
        }
        const alpha = 0.58;
        prev.x = prev.x + alpha * (nextX - prev.x);
        prev.y = prev.y + alpha * (nextY - prev.y);
        prev.yaw = prev.yaw + alpha * (nextYaw - prev.yaw);
        prev.length = prev.length + alpha * (nextLength - prev.length);
        prev.width = prev.width + alpha * (nextWidth - prev.width);
        prev.score = Number(det.score || prev.score);
        prev.class_id = det.class_id || prev.class_id;
        prev.lastSeenMs = now;
      }
      for (const key of Object.keys(tracks)) {
        if (!activeKeys.has(key) && now - tracks[key].lastSeenMs > 1500) {
          delete tracks[key];
        }
      }
      drawDetectionBoxes(ctx, Object.values(tracks), mapToScreen, now);
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

    if (!followPose && overlay?.robot_pose) {
      const rp = mapToScreen(Number(overlay.robot_pose.x), Number(overlay.robot_pose.y));
      const offscreen = rp.x < -80 || rp.y < -80 || rp.x > w + 80 || rp.y > h + 80;
      if (offscreen) {
        ctx.fillStyle = "rgba(44, 23, 8, 0.8)";
        ctx.fillRect(14, 74, Math.min(360, w - 28), 38);
        ctx.fillStyle = "#ffd5ad";
        ctx.font = "600 12px 'IBM Plex Sans', sans-serif";
        ctx.fillText("Robot is outside viewport. Use Recenter.", 24, 98);
      }
    }

    if (
      (!layers.map || !mapImageRef.current) &&
      (!layers.obstacle || !obstacleImageRef.current) &&
      (!layers.scan || !scanPointsPresent(overlay, manualExtrinsics)) &&
      (!layers.plan || !(overlay?.path_points?.length || 0)) &&
      (!layers.detections || !(detections?.detections?.length || 0))
    ) {
      ctx.fillStyle = "rgba(7, 16, 28, 0.75)";
      ctx.fillRect(14, 14, Math.min(410, w - 28), 54);
      ctx.fillStyle = "#cddff7";
      ctx.font = "600 13px 'IBM Plex Sans', sans-serif";
      ctx.fillText("No active visual layers yet. Check topic publishers/TF.", 24, 35);
      ctx.font = "500 12px 'IBM Plex Sans', sans-serif";
      ctx.fillStyle = "#9bb4d5";
      ctx.fillText("The view remains active and will render as soon as data arrives.", 24, 54);
    }

    if (showTfOverlay && tfChain) {
      drawTfOverlay(ctx, w, tfChain);
    }

    if (showAxesOverlay) {
      drawAxesOverlay(ctx, h, overlay?.fixed_frame || overlay?.map_frame || tfChain?.fixed_frame || "frame");
    }
  }, [
    detections,
    followPose,
    layers,
    manualExtrinsics,
    map,
    obstacleMap,
    onRenderStats,
    overlay,
    showAxesOverlay,
    showTfOverlay,
    tfChain,
    view,
  ]);

  return (
    <canvas
      ref={canvasRef}
      width={width}
      height={height}
      className="map-canvas"
      onContextMenu={(event) => event.preventDefault()}
      onPointerDown={(event) => {
        if (!panEnabled) {
          return;
        }
        const canvas = canvasRef.current;
        if (!canvas) {
          return;
        }
        canvas.setPointerCapture(event.pointerId);
        dragRef.current = {
          pointerId: event.pointerId,
          startX: event.clientX,
          startY: event.clientY,
          originPanX: view.panX,
          originPanY: view.panY,
        };
        onPanStart?.();
      }}
      onPointerMove={(event) => {
        if (!panEnabled) {
          return;
        }
        const drag = dragRef.current;
        if (!drag || drag.pointerId !== event.pointerId) {
          return;
        }
        const dx = event.clientX - drag.startX;
        const dy = event.clientY - drag.startY;
        setView((prev) => ({
          ...prev,
          panX: drag.originPanX + dx,
          panY: drag.originPanY + dy,
        }));
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
        setView((prev) => {
          const zoomNext = Math.max(10, Math.min(130, prev.zoom * (event.deltaY < 0 ? 1.08 : 0.92)));
          return {
            ...prev,
            zoom: zoomNext,
          };
        });
      }}
    />
  );
}

function scanPointsPresent(
  overlay: MapOverlayPayload | null,
  manualExtrinsics: ManualExtrinsics,
): boolean {
  if ((overlay?.scan_points?.length || 0) > 0) {
    return true;
  }
  if (!manualExtrinsics.enabled) {
    return (overlay?.scan_points_sensor?.length || 0) > 0;
  }
  return applyManualTransform((overlay?.scan_points_sensor || []) as ScanPoint[], manualExtrinsics).length > 0;
}
