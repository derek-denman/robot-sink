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
  sample_count: number;
  angle_min: number;
  angle_max: number;
  angle_increment: number;
  range_min: number;
  range_max: number;
  point_count: number;
  points: ScanPoint[];
  ranges_preview?: number[];
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

export type ObstacleValueMapping = {
  free: number;
  occupied: number;
  unknown: number;
  inflation_min: number;
  inflation_max: number;
};

export type ObstacleMapPayload = {
  stamp_unix: number;
  topic: string;
  source: "nav2" | "computed" | "none";
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
  value_mapping: ObstacleValueMapping;
  depth_fusion_enabled: boolean;
  warnings: string[];
};

export type DetectionBev = {
  id: string;
  class_id: string;
  score: number;
  distance_m?: number;
  bev: {
    x: number;
    y: number;
    yaw: number;
    length: number;
    width: number;
  };
  camera_marker?: {
    u: number;
    v: number;
    u_norm: number;
    v_norm: number;
    rect_w: number;
    rect_h: number;
    frame_width: number;
    frame_height: number;
  } | null;
};

export type DetectionPayload = {
  stamp_unix: number;
  topic: string;
  frame_id: string;
  fixed_frame: string;
  resolved_in_fixed: boolean;
  warnings: string[];
  detection_count: number;
  detections: Array<{
    id: string;
    class_id: string;
    score: number;
    distance_m?: number;
    position?: { x: number; y: number; z: number };
    size?: { x: number; y: number; z: number };
    yaw?: number;
    render_frame?: string;
    bev: DetectionBev["bev"];
    camera_marker?: DetectionBev["camera_marker"];
  }>;
};

export type DepthPayload = {
  stamp_unix: number;
  topic: string;
  frame_id: string;
  encoding: string;
  width: number;
  height: number;
  rows: number;
  cols: number;
  step_x?: number;
  step_y?: number;
  min_mm: number | null;
  max_mm: number | null;
  data: number[];
};

export type MapOverlayPayload = {
  stamp_unix: number;
  map_frame: string;
  fixed_frame?: string;
  selected_fixed_frame?: string;
  fixed_frame_warnings?: string[];
  scan_topic: string;
  scan_frame?: string;
  scan_points: ScanPoint[];
  scan_points_sensor?: ScanPoint[];
  scan_point_count: number;
  scan_sensor_point_count?: number;
  scan_resolved?: boolean;
  scan_age_sec: number | null;
  robot_pose: {
    x: number;
    y: number;
    yaw: number;
    stamp_unix: number;
    frame_id: string;
    base_frame: string;
    source?: "tf" | "odom" | "origin";
  } | null;
  path_topic: string;
  path_frame?: string;
  path_points: ScanPoint[];
  path_point_count: number;
  path_resolved?: boolean;
  planner_active?: boolean;
  active_plan_topics?: string[];
  detection_topic?: string;
  detection_frame?: string;
  detection_count?: number;
  detection_resolved?: boolean;
  detections?: DetectionBev[];
  drivable_wedge?: Array<ScanPoint>;
  warnings?: string[];
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
  selected_rgb_topic?: string;
  available_topics: string[];
  topic_metrics: CameraMetric[];
  publisher_count?: number;
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
  sample_count?: number;
  frame_id?: string | null;
  range_min?: number;
  range_max?: number;
  angle_min?: number;
  angle_max?: number;
  publisher_count?: number;
  connected: boolean;
};

export type FixedFrameState = {
  selected: string;
  resolved: string;
  available: string[];
  options: string[];
  warnings: string[];
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
    source?: "tf" | "odom" | "origin";
  } | null;
  selected_path_topic: string;
  available_map_topics: string[];
  available_path_topics: string[];
  active_plan_topics?: string[];
  planner_active?: boolean;
  fixed_frame?: FixedFrameState;
  path_rate_hz: number;
};

export type ObstacleMapStreamStatus = {
  selected_topic: string;
  available_topics: string[];
  source: "nav2" | "computed" | "none";
  fps: number;
  age_sec: number | null;
  connected: boolean;
  frame_id: string;
  width: number;
  height: number;
  resolution: number;
  publisher_count: number;
  depth_fusion_enabled: boolean;
  value_mapping: ObstacleValueMapping;
  warnings: string[];
};

export type PointcloudStreamStatus = {
  selected_topic: string;
  available_topics: string[];
  fps: number;
  age_sec: number | null;
  connected: boolean;
  publisher_count?: number;
  frame_id: string | null;
  point_count: number;
};

export type RgbRawStatus = {
  topic: string;
  fps: number;
  age_sec: number | null;
  connected: boolean;
  publisher_count?: number;
  frame_id?: string | null;
  encoding?: string | null;
  width?: number | null;
  height?: number | null;
  camera_info_topic?: string | null;
  camera_info_age_sec?: number | null;
};

export type DepthStreamStatus = {
  topic: string;
  fps: number;
  age_sec: number | null;
  connected: boolean;
  publisher_count?: number;
  frame_id?: string | null;
  encoding?: string | null;
  width?: number | null;
  height?: number | null;
  rows?: number;
  cols?: number;
  min_mm?: number | null;
  max_mm?: number | null;
};

export type DetectionStreamStatus = {
  topic: string;
  available_topics: string[];
  fps: number;
  age_sec: number | null;
  connected: boolean;
  publisher_count?: number;
  frame_id?: string | null;
  detection_count?: number;
  resolved_in_fixed?: boolean;
  warnings?: string[];
  vision_msgs_available?: boolean;
};

export type TfHealthStatus = {
  tf_age_sec: number | null;
  tf_static_age_sec: number | null;
  fixed_frame: FixedFrameState;
  scan_to_fixed_ok: boolean;
  detections_to_fixed_ok: boolean;
  missing_pairs: string[];
  has_base_link: boolean;
  has_odom_base_tf: boolean;
  has_base_laser_tf: boolean;
  has_base_oak_tf: boolean;
};

export type TopicOption = {
  topic: string;
  publisher_count: number;
};

export type TopicCatalog = {
  scan: { selected_topic: string; options: TopicOption[] };
  rgb: { selected_topic: string; options: TopicOption[] };
  depth: { selected_topic: string; options: TopicOption[] };
  detections: { selected_topic: string; options: TopicOption[] };
  map: { selected_topic: string; options: TopicOption[] };
  costmap: { selected_topic: string; options: TopicOption[] };
  pointcloud: { selected_topic: string; options: TopicOption[] };
  plans: { selected_topic: string; options: TopicOption[] };
  fixed_frame: FixedFrameState;
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
    depth_rate_hz?: number;
    detections_rate_hz?: number;
    camera_stream_rate_hz: number;
    scan_age_sec: number | null;
    odom_age_sec: number | null;
    camera_age_sec: number | null;
    depth_age_sec?: number | null;
    detections_age_sec?: number | null;
    camera_stream_age_sec: number | null;
    scan_stream_connected: boolean;
    camera_stream_connected: boolean;
    tf_health?: TfHealthStatus;
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
    rgb: RgbRawStatus;
    depth: DepthStreamStatus;
    detections: DetectionStreamStatus;
    map: MapStreamStatus;
    obstacle_map: ObstacleMapStreamStatus;
    pointcloud: PointcloudStreamStatus;
    tf: TfHealthStatus;
    topic_catalog: TopicCatalog;
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
  | { type: "obstacle_map"; data: ObstacleMapPayload }
  | { type: "map_overlay"; data: MapOverlayPayload }
  | { type: "pointcloud"; data: PointcloudPayload }
  | { type: "depth"; data: DepthPayload }
  | { type: "detections"; data: DetectionPayload }
  | { type: string; data: Record<string, unknown> };
