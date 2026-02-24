import { Card, Tag, Typography } from "antd";
import { hz, secAge } from "../../lib/format";
import type { StatusPayload } from "../../types";

const { Text } = Typography;

type TelemetryColumnProps = {
  wsConnected: boolean;
  status: StatusPayload | null;
};

function odomHealthy(status: StatusPayload | null): boolean {
  if (!status) {
    return false;
  }
  const tf = status.visualizer?.tf;
  return Boolean(tf?.has_odom_base_tf && tf?.scan_to_fixed_ok && (tf?.fixed_frame?.resolved || "") === "odom");
}

export function TelemetryColumn({ wsConnected, status }: TelemetryColumnProps): JSX.Element {
  const sensor = status?.visualizer;
  const tfFix = odomHealthy(status);

  return (
    <div className="unified-telemetry-column">
      <Card className="hmi-mini-card unified-mini-card" title="Live Telemetry" size="small">
        <Card className="hmi-mini-card unified-mini-card-inner" title="Connection" size="small">
          <div className="telemetry-row">
            <Text>WebSocket:</Text>
            <Tag color={wsConnected ? "success" : "error"}>{wsConnected ? "online" : "offline"}</Tag>
          </div>
          <div className="telemetry-row">
            <Text>WS clients:</Text>
            <Text>{status?.connection?.ws_clients || 0}</Text>
          </div>
        </Card>

        <Card className="hmi-mini-card unified-mini-card-inner" title="Sensors" size="small">
          <div className="unified-sensor-grid">
            <div>
              <div className="sensor-label">Lidar</div>
              <div className="sensor-value">{hz(sensor?.scan?.fps)}</div>
            </div>
            <div>
              <div className="sensor-label">OAK-D</div>
              <div className="sensor-value">{hz(sensor?.camera?.fps)}</div>
            </div>
            <div>
              <div className="sensor-label">Map</div>
              <div className="sensor-value">{hz(sensor?.map?.fps)}</div>
            </div>
            <div>
              <div className="sensor-label">PointCloud</div>
              <div className="sensor-value">{hz(sensor?.pointcloud?.fps)}</div>
            </div>
          </div>
          <div className="telemetry-age">scan age: {secAge(sensor?.scan?.age_sec)}</div>
          <div className="telemetry-age">map age: {secAge(sensor?.map?.age_sec)}</div>
        </Card>

        <Card className="hmi-mini-card unified-mini-card-inner" title="TF / Odom" size="small">
          <Tag color={tfFix ? "success" : "warning"}>{tfFix ? "odom_fix" : "tf_warn"}</Tag>
        </Card>

        <Card className="hmi-mini-card unified-mini-card-inner" title="Key Hints" size="small">
          <div className="hint-line"><strong>Base:</strong> ARROWS + SPACE, SHIFT/CTRL/X</div>
          <div className="hint-line"><strong>Arm:</strong> 1-6, [ ], O P, H N B, ESC</div>
        </Card>
      </Card>
    </div>
  );
}
