import type { DetectionPayload } from "../types";

type Props = {
  detections: DetectionPayload | null;
  className?: string;
};

export function CameraDetectionOverlay({ detections, className = "" }: Props): JSX.Element {
  const items = detections?.detections || [];
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
        return (
          <g key={`${item.id}-${cx.toFixed(4)}-${cy.toFixed(4)}`}>
            {rw > 0.001 && rh > 0.001 ? (
              <rect
                x={x}
                y={y}
                width={rw}
                height={rh}
                fill="rgba(255, 62, 204, 0.14)"
                stroke="rgba(255, 74, 210, 0.95)"
                strokeWidth={0.003}
              />
            ) : null}
            <circle cx={cx} cy={cy} r={0.006} fill="rgba(255, 122, 227, 0.95)" />
            <text
              x={cx + 0.01}
              y={cy - 0.012}
              fontSize="0.022"
              fontWeight={600}
              fill="rgba(255, 182, 240, 0.95)"
            >
              {label}
            </text>
          </g>
        );
      })}
    </svg>
  );
}
