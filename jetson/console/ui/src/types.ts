export type RobotMode =
  | "manual"
  | "commissioning"
  | "mapping"
  | "nav"
  | "pickplace"
  | "training"
  | "reliability"
  | "demo";

export type SafetySnapshot = {
  armed: boolean;
  estop_latched: boolean;
  watchdog_ok: boolean;
  command_timeout_sec: number;
  last_command_age_sec: number | null;
  last_disarm_reason: string;
};

export type ScanPoint = [number, number];

export type ScanPayload = {
  stamp_unix: number;
  frame_id: string;
  range_min: number;
  range_max: number;
  point_count: number;
  points: ScanPoint[];
};

export type CameraMetric = {
  topic: string;
  fps: number;
  age_sec: number | null;
  connected: boolean;
  format?: string | null;
};

export type CameraStreamStatus = {
  selected_topic: string;
  available_topics: string[];
  topic_metrics: CameraMetric[];
  fps: number;
  age_sec: number | null;
  connected: boolean;
  format?: string | null;
};

export type ScanStreamStatus = {
  topic: string;
  fps: number;
  age_sec: number | null;
  point_count: number;
  connected: boolean;
};

export type ArmJointConfig = {
  name: string;
  min: number;
  max: number;
  step: number;
  default?: number;
};

export type LogSource = {
  id: string;
  path: string | null;
  exists: boolean;
  dynamic: boolean;
};

export type StatusPayload = {
  timestamp_unix: number;
  mode: RobotMode;
  safety: SafetySnapshot;
  health: {
    base_connected: boolean;
    tf_ok: boolean;
    watchdog_status: string;
    bag_active: boolean;
    lidar_rate_hz: number;
    odom_rate_hz: number;
    oak_rate_hz: number;
    camera_stream_rate_hz: number;
    scan_age_sec: number | null;
    odom_age_sec: number | null;
    camera_age_sec: number | null;
    camera_stream_age_sec: number | null;
    scan_stream_connected: boolean;
    camera_stream_connected: boolean;
    nav: {
      state: string;
      last_result: string;
      active: boolean;
    };
  };
  dashboard: {
    base_connected: boolean;
    last_encoder_update_unix: number | null;
    topic_rates: Record<string, number>;
    nav_state: string;
    temperatures: number[];
  };
  navigation: {
    state: string;
    last_result: string;
    active: boolean;
    feedback: Record<string, unknown>;
  };
  demo: {
    active: boolean;
    target_count: number;
    completed_count: number;
  };
  mapping: {
    slam_active: boolean;
    localization_mode: boolean;
    last_saved_map: string | null;
  };
  recording: {
    status: {
      active: string | null;
      storage_path: string;
    };
    recent: Array<{
      path: string;
      name: string;
      mtime_unix: number;
      size_bytes: number;
    }>;
  };
  reliability: {
    topic_rates: Record<string, number>;
    temperatures_c: number[];
    cpu_temp_c: number | null;
    max_temp_c: number | null;
    disk_total_bytes: number;
    disk_used_bytes: number;
    disk_free_bytes: number;
  };
  visualizer: {
    scan: ScanStreamStatus;
    camera: CameraStreamStatus;
  };
  arm: {
    joints: ArmJointConfig[];
    named_poses: string[];
  };
  logs: {
    sources: LogSource[];
  };
  connection: {
    ws_clients: number;
  };
  foxglove: {
    port: number;
    url_template: string;
  };
  capabilities: Record<string, boolean | string>;
};

export type ApiResult<T = unknown> = {
  ok: boolean;
  error?: string;
  result?: T;
} & Record<string, unknown>;

export type WsEvent =
  | { type: "status"; data: StatusPayload }
  | { type: "scan"; data: ScanPayload }
  | { type: string; data: Record<string, unknown> };
