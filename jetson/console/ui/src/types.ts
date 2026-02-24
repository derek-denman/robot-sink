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
export type Point3 = [number, number, number];

export type ScanPayload = {
  stamp_unix: number;
  frame_id: string;
  range_min: number;
  range_max: number;
  point_count: number;
  points: ScanPoint[];
  map_frame?: string;
  point_count_map?: number;
  points_map?: ScanPoint[];
};

export type MapPayload = {
  stamp_unix: number;
  frame_id: string;
  width: number;
  height: number;
  resolution: number;
  origin: {
    x: number;
    y: number;
    yaw: number;
  };
  encoding: "rle_u8" | string;
  cell_count: number;
  data_rle: Array<[number, number]>;
};

export type MapOverlayPayload = {
  stamp_unix: number;
  map_frame: string;
  scan_topic: string;
  scan_points: ScanPoint[];
  scan_point_count: number;
  scan_age_sec: number | null;
  robot_pose: {
    x: number;
    y: number;
    yaw: number;
    stamp_unix: number;
    frame_id: string;
    base_frame: string;
  } | null;
  path_topic: string;
  path_points: ScanPoint[];
  path_point_count: number;
  tf_age_sec: number | null;
};

export type PointcloudPayload = {
  stamp_unix: number;
  topic: string;
  frame_id: string;
  point_count: number;
  source_points?: number;
  points: Point3[];
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

export type MapStreamStatus = {
  topic: string;
  fps: number;
  age_sec: number | null;
  connected: boolean;
  map_available: boolean;
  frame_id: string;
  width: number;
  height: number;
  resolution: number;
  tf_age_sec: number | null;
  scan_rate_hz: number;
  pose: {
    x: number;
    y: number;
    yaw: number;
    stamp_unix: number;
    frame_id: string;
    base_frame: string;
  } | null;
  selected_path_topic: string;
  available_map_topics: string[];
  available_path_topics: string[];
  path_rate_hz: number;
};

export type PointcloudStreamStatus = {
  selected_topic: string;
  available_topics: string[];
  fps: number;
  age_sec: number | null;
  connected: boolean;
  frame_id: string | null;
  point_count: number;
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
    map: MapStreamStatus;
    pointcloud: PointcloudStreamStatus;
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
  | { type: "map"; data: MapPayload }
  | { type: "map_overlay"; data: MapOverlayPayload }
  | { type: "pointcloud"; data: PointcloudPayload }
  | { type: string; data: Record<string, unknown> };
