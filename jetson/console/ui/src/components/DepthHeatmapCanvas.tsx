import { useEffect, useRef } from "react";
import type { DepthPayload } from "../types";

type Props = {
  depth: DepthPayload | null;
  className?: string;
};

function colorFor(value: number): string {
  const v = Math.max(0, Math.min(255, value));
  const hue = 220 - (v / 255) * 210;
  return `hsla(${hue}, 100%, 56%, 0.38)`;
}

export function DepthHeatmapCanvas({ depth, className = "" }: Props): JSX.Element {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);

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
    ctx.clearRect(0, 0, width, height);

    if (!depth || !depth.data?.length || !depth.rows || !depth.cols) {
      return;
    }

    const cellW = width / Math.max(1, depth.cols);
    const cellH = height / Math.max(1, depth.rows);
    for (let r = 0; r < depth.rows; r += 1) {
      for (let c = 0; c < depth.cols; c += 1) {
        const idx = r * depth.cols + c;
        if (idx >= depth.data.length) {
          continue;
        }
        const value = Number(depth.data[idx] || 0);
        if (value <= 0) {
          continue;
        }
        ctx.fillStyle = colorFor(value);
        ctx.fillRect(c * cellW, r * cellH, cellW + 1, cellH + 1);
      }
    }
  }, [depth]);

  return <canvas ref={canvasRef} className={className} width={1280} height={720} />;
}
