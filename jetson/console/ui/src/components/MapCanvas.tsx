import { useEffect, useMemo, useRef, useState } from "react";
import type { MapOverlayPayload, MapPayload, ScanPoint } from "../types";

type Props = {
  map: MapPayload | null;
  overlay: MapOverlayPayload | null;
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

function decodeRle(payload: MapPayload | null): Uint8Array | null {
  if (!payload || payload.width <= 0 || payload.height <= 0) {
    return null;
  }
  if (payload.encoding !== "rle_u8" || !Array.isArray(payload.data_rle)) {
    return null;
  }
  const total = payload.width * payload.height;
  const out = new Uint8Array(total);
  let idx = 0;
  for (const run of payload.data_rle) {
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

function drawRobotArrow(
  ctx: CanvasRenderingContext2D,
  mapToScreen: (x: number, y: number) => { x: number; y: number },
  x: number,
  y: number,
  yaw: number,
): void {
  const c = mapToScreen(x, y);
  const length = 0.35;
  const width = 0.2;
  const tip = mapToScreen(x + Math.cos(yaw) * length, y + Math.sin(yaw) * length);
  const left = mapToScreen(
    x + Math.cos(yaw + Math.PI * 0.7) * width,
    y + Math.sin(yaw + Math.PI * 0.7) * width,
  );
  const right = mapToScreen(
    x + Math.cos(yaw - Math.PI * 0.7) * width,
    y + Math.sin(yaw - Math.PI * 0.7) * width,
  );

  ctx.beginPath();
  ctx.moveTo(tip.x, tip.y);
  ctx.lineTo(left.x, left.y);
  ctx.lineTo(right.x, right.y);
  ctx.closePath();
  ctx.fillStyle = "#5ad8ff";
  ctx.fill();
  ctx.lineWidth = 1.4;
  ctx.strokeStyle = "rgba(17, 44, 66, 0.9)";
  ctx.stroke();

  ctx.beginPath();
  ctx.arc(c.x, c.y, 4, 0, Math.PI * 2);
  ctx.fillStyle = "#84ffcf";
  ctx.fill();
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
      continue;
    }
    ctx.lineTo(p.x, p.y);
  }
  if (!started) {
    return;
  }
  ctx.lineWidth = 2.0;
  ctx.strokeStyle = "rgba(246, 198, 83, 0.95)";
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
  ctx.fillStyle = "#22d3ee";
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

export function MapCanvas({ map, overlay }: Props): JSX.Element {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const dragRef = useRef<DragState | null>(null);
  const mapImageRef = useRef<HTMLCanvasElement | null>(null);
  const [view, setView] = useState<ViewState>({ zoom: 36, panX: 0, panY: 0 });

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

        let shade = 110;
        if (cell === 255) {
          shade = 80;
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

    ctx.fillStyle = "#08111a";
    ctx.fillRect(0, 0, width, height);

    const mapToScreen = (x: number, y: number): { x: number; y: number } => ({
      x: width * 0.5 + view.panX + x * view.zoom,
      y: height * 0.5 + view.panY - y * view.zoom,
    });

    const drawGrid = () => {
      const spacing = Math.max(18, view.zoom);
      ctx.strokeStyle = "rgba(88, 120, 155, 0.18)";
      ctx.lineWidth = 1;
      for (let x = width * 0.5 + (view.panX % spacing); x <= width; x += spacing) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, height);
        ctx.stroke();
      }
      for (let x = width * 0.5 + (view.panX % spacing); x >= 0; x -= spacing) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, height);
        ctx.stroke();
      }
      for (let y = height * 0.5 + (view.panY % spacing); y <= height; y += spacing) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(width, y);
        ctx.stroke();
      }
      for (let y = height * 0.5 + (view.panY % spacing); y >= 0; y -= spacing) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(width, y);
        ctx.stroke();
      }
    };

    drawGrid();

    if (map && mapImageRef.current) {
      const originX = Number(map.origin?.x || 0);
      const originY = Number(map.origin?.y || 0);
      const mapWidthM = Number(map.width || 0) * Number(map.resolution || 0);
      const mapHeightM = Number(map.height || 0) * Number(map.resolution || 0);
      const topLeft = mapToScreen(originX, originY + mapHeightM);
      const bottomRight = mapToScreen(originX + mapWidthM, originY);
      const drawW = bottomRight.x - topLeft.x;
      const drawH = bottomRight.y - topLeft.y;

      ctx.imageSmoothingEnabled = false;
      ctx.globalAlpha = 0.95;
      ctx.drawImage(mapImageRef.current, topLeft.x, topLeft.y, drawW, drawH);
      ctx.globalAlpha = 1.0;
      ctx.strokeStyle = "rgba(138, 184, 223, 0.5)";
      ctx.strokeRect(topLeft.x, topLeft.y, drawW, drawH);
    } else {
      ctx.fillStyle = "#97a7b9";
      ctx.font = "600 15px 'IBM Plex Sans', sans-serif";
      ctx.fillText("Waiting for /map occupancy grid...", 20, 30);
    }

    drawPolyline(ctx, overlay?.path_points || [], mapToScreen);
    drawScanPoints(ctx, overlay?.scan_points || [], mapToScreen);
    if (overlay?.robot_pose) {
      drawRobotArrow(
        ctx,
        mapToScreen,
        Number(overlay.robot_pose.x),
        Number(overlay.robot_pose.y),
        Number(overlay.robot_pose.yaw),
      );
    }
  }, [map, overlay, view]);

  return (
    <canvas
      ref={canvasRef}
      width={860}
      height={520}
      className="map-canvas"
      onContextMenu={(event) => event.preventDefault()}
      onPointerDown={(event) => {
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
      }}
      onPointerMove={(event) => {
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
        const canvas = canvasRef.current;
        if (!canvas) {
          return;
        }
        const rect = canvas.getBoundingClientRect();
        const px = event.clientX - rect.left;
        const py = event.clientY - rect.top;

        setView((prev) => {
          const zoomNext = Math.max(8, Math.min(180, prev.zoom * (event.deltaY < 0 ? 1.12 : 0.89)));
          if (Math.abs(zoomNext - prev.zoom) < 1e-6) {
            return prev;
          }

          const worldX = (px - canvas.width * 0.5 - prev.panX) / prev.zoom;
          const worldY = -(py - canvas.height * 0.5 - prev.panY) / prev.zoom;

          const panX = px - canvas.width * 0.5 - worldX * zoomNext;
          const panY = py - canvas.height * 0.5 + worldY * zoomNext;

          return {
            zoom: zoomNext,
            panX,
            panY,
          };
        });
      }}
    />
  );
}
