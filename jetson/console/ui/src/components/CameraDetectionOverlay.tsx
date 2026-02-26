import type { DetectionPayload } from "../types";

type Props = {
  detections: DetectionPayload | null;
  className?: string;
};

export function CameraDetectionOverlay({ detections, className = "" }: Props): JSX.Element {
  const items = detections?.detections || [];

  const colorFor = (classId: string): string => {
    let hash = 0;
    for (let i = 0; i < classId.length; i += 1) {
      hash = (hash << 5) - hash + classId.charCodeAt(i);
      hash |= 0;
    }
    const hue = Math.abs(hash) % 360;
    return `hsl(${hue}, 92%, 62%)`;
  };

  return (
    <svg className={className} viewBox="0 0 1 1" preserveAspectRatio="none">
      {items.map((item) => {
        const marker = item.camera_marker;
        if (!marker) {
          return null;
        }
        const cx = Number(marker.u_norm || 0);
        const cy = Number(marker.v_norm || 0);
        const rw = Number(marker.rect_w || 0) / Math.max(1, Number(marker.frame_width || 1));
        const rh = Number(marker.rect_h || 0) / Math.max(1, Number(marker.frame_height || 1));
        const x = cx - rw * 0.5;
        const y = cy - rh * 0.5;
        const label = `${item.class_id || item.id} ${(Number(item.score || 0) * 100).toFixed(0)}%`;
        const color = colorFor(String(item.class_id || item.id || "obj"));
        const labelW = Math.max(0.06, Math.min(0.28, label.length * 0.012));
        return (
          <g key={`${item.id}-${cx.toFixed(4)}-${cy.toFixed(4)}`}>
            {rw > 0.001 && rh > 0.001 ? (
              <rect
                x={x}
                y={y}
                width={rw}
                height={rh}
                rx={0.004}
                fill={`${color}33`}
                stroke={color}
                strokeWidth={0.003}
              />
            ) : null}
            <rect
              x={Math.max(0.001, Math.min(0.999 - labelW, cx + 0.006))}
              y={Math.max(0.001, cy - 0.034)}
              width={labelW}
              height={0.024}
              rx={0.004}
              fill="rgba(9, 18, 30, 0.78)"
              stroke={color}
              strokeWidth={0.0015}
            />
            <circle cx={cx} cy={cy} r={0.006} fill={color} />
            <text
              x={Math.max(0.004, Math.min(0.994, cx + 0.012))}
              y={Math.max(0.018, cy - 0.016)}
              fontSize="0.018"
              fontWeight={600}
              fill="rgba(226, 241, 255, 0.98)"
            >
              {label}
            </text>
          </g>
        );
      })}
    </svg>
  );
}
