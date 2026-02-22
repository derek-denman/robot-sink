import { useEffect, useRef } from "react";
import type { ScanPayload } from "../types";

type Props = {
  scan: ScanPayload | null;
};

export function LidarCanvas({ scan }: Props): JSX.Element {
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
    const cx = width / 2;
    const cy = height / 2;

    ctx.fillStyle = "#0b1118";
    ctx.fillRect(0, 0, width, height);

    ctx.strokeStyle = "rgba(134, 187, 255, 0.22)";
    ctx.lineWidth = 1;
    for (let ring = 1; ring <= 5; ring += 1) {
      const radius = ring * (Math.min(width, height) * 0.09);
      ctx.beginPath();
      ctx.arc(cx, cy, radius, 0, Math.PI * 2);
      ctx.stroke();
    }

    ctx.strokeStyle = "rgba(163, 196, 235, 0.24)";
    ctx.beginPath();
    ctx.moveTo(cx, 0);
    ctx.lineTo(cx, height);
    ctx.moveTo(0, cy);
    ctx.lineTo(width, cy);
    ctx.stroke();

    if (!scan) {
      return;
    }

    const plotRange = Math.max(1, Math.min(25, Number(scan.range_max || 8)));
    const scale = (Math.min(width, height) * 0.44) / plotRange;

    ctx.fillStyle = "#36d1dc";
    for (const point of scan.points || []) {
      if (!Array.isArray(point) || point.length < 2) {
        continue;
      }
      const x = Number(point[0]);
      const y = Number(point[1]);
      if (!Number.isFinite(x) || !Number.isFinite(y)) {
        continue;
      }
      const px = cx + y * scale;
      const py = cy - x * scale;
      if (px < 0 || py < 0 || px > width || py > height) {
        continue;
      }
      ctx.fillRect(px, py, 2, 2);
    }
  }, [scan]);

  return <canvas ref={canvasRef} width={680} height={420} style={{ width: "100%", borderRadius: 10 }} />;
}
