import {
  Alert,
  Badge,
  Button,
  Card,
  Collapse,
  Col,
  Divider,
  Empty,
  Input,
  InputNumber,
  Layout,
  List,
  Menu,
  Progress,
  Row,
  Segmented,
  Select,
  Slider,
  Space,
  Statistic,
  Switch,
  Table,
  Tag,
  Typography,
  message,
} from "antd";
import {
  ApiOutlined,
  CameraOutlined,
  ControlOutlined,
  DashboardOutlined,
  ExperimentOutlined,
  PoweroffOutlined,
  RadarChartOutlined,
  RobotOutlined,
  SafetyCertificateOutlined,
  ToolOutlined,
  WarningOutlined,
} from "@ant-design/icons";
import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { LidarCanvas } from "./components/LidarCanvas";
import { FusedViewCanvas } from "./components/FusedViewCanvas";
import { CameraDetectionOverlay } from "./components/CameraDetectionOverlay";
import { DepthHeatmapCanvas } from "./components/DepthHeatmapCanvas";
import { UnifiedControlPage } from "./components/unified/UnifiedControlPage";
import { getStatus, postApi } from "./lib/api";
import { agoUnix, bytes, hz, secAge, timestampToLocal } from "./lib/format";
import { getCachedCameraTopic, getCachedMode, useConsoleStore } from "./store/useConsoleStore";
import type {
  ArmJointConfig,
  DepthPayload,
  DetectionPayload,
  LogSource,
  MapOverlayPayload,
  ObstacleMapPayload,
  MapPayload,
  PointcloudPayload,
  RobotMode,
  ScanPayload,
  StatusPayload,
  WsEvent,
} from "./types";

const { Header, Content, Sider } = Layout;
const { Title, Text } = Typography;

type MenuKey = "operations" | "visualizer" | "manual" | "unified" | "arm" | "training" | "demo" | "logs";
type SeverityFilter = "all" | "info" | "warn" | "error";

const MODE_OPTIONS: Array<{ value: RobotMode; label: string }> = [
  { value: "manual", label: "Manual" },
  { value: "commissioning", label: "Commissioning" },
  { value: "mapping", label: "Mapping" },
  { value: "nav", label: "Navigation" },
  { value: "pickplace", label: "Pick & Place" },
  { value: "training", label: "Training" },
  { value: "reliability", label: "Reliability" },
  { value: "demo", label: "Demo" },
];

const DRIVE_VECTORS: Record<string, { linear_x: number; angular_z: number }> = {
  forward: { linear_x: 1, angular_z: 0 },
  backward: { linear_x: -1, angular_z: 0 },
  left: { linear_x: 0, angular_z: 1 },
  right: { linear_x: 0, angular_z: -1 },
};

type LogEntry = {
  ts: number;
  source: string;
  line: string;
  severity: SeverityFilter;
};

const VIS_SETTINGS_KEY = "robot_console_hmi.visualizer_settings_v2";

function classifySeverity(line: string): SeverityFilter {
  const lower = line.toLowerCase();
  if (lower.includes("error") || lower.includes("fatal") || lower.includes("traceback")) {
    return "error";
  }
  if (lower.includes("warn") || lower.includes("timeout") || lower.includes("unavailable")) {
    return "warn";
  }
  return "info";
}

function isEditableElement(target: EventTarget | null): boolean {
  if (!(target instanceof HTMLElement)) {
    return false;
  }
  return ["INPUT", "TEXTAREA", "SELECT"].includes(target.tagName) || target.isContentEditable;
}

export default function App(): JSX.Element {
  const {
    transport,
    robot,
    ui,
    setWsConnected,
    setWsStateLabel,
    setStatus,
    setScanPayload,
    setMapPayload,
    setObstacleMapPayload,
    setMapOverlay,
    setPointcloudPayload,
    setDepthPayload,
    setDetectionPayload,
    setSelectedMode,
    setSelectedCameraTopic,
    setModeReconciled,
    setNavKey,
    toggleRightPanel,
  } = useConsoleStore();

  const [messageApi, contextHolder] = message.useMessage();
  const [modeDraft, setModeDraft] = useState<RobotMode>(robot.selectedMode);
  const [modeDirty, setModeDirty] = useState(false);
  const [cameraDraft, setCameraDraft] = useState<string>(robot.selectedCameraTopic);
  const [cameraReconciled, setCameraReconciled] = useState(false);
  const [mapTopicDraft, setMapTopicDraft] = useState("");
  const [pathTopicDraft, setPathTopicDraft] = useState("");
  const [costmapTopicDraft, setCostmapTopicDraft] = useState("");
  const [pointcloudDraft, setPointcloudDraft] = useState("");
  const [scanTopicDraft, setScanTopicDraft] = useState("");
  const [rgbTopicDraft, setRgbTopicDraft] = useState("");
  const [depthTopicDraft, setDepthTopicDraft] = useState("");
  const [detectionTopicDraft, setDetectionTopicDraft] = useState("");
  const [fixedFrameDraft, setFixedFrameDraft] = useState("auto");
  const [showDepthBlend, setShowDepthBlend] = useState(false);
  const [showDetections, setShowDetections] = useState(true);
  const [showMapLayer, setShowMapLayer] = useState(true);
  const [showScanLayer, setShowScanLayer] = useState(true);
  const [showPlanLayer, setShowPlanLayer] = useState(true);
  const [showTfChainOverlay, setShowTfChainOverlay] = useState(false);
  const [showAxesOverlay, setShowAxesOverlay] = useState(true);
  const [mainFollowMode, setMainFollowMode] = useState(true);
  const [fusedRecenterToken, setFusedRecenterToken] = useState(0);
  const [mainRenderStats, setMainRenderStats] = useState({ fps: 0, droppedFrames: 0 });
  const [miniRenderStats, setMiniRenderStats] = useState({ fps: 0, droppedFrames: 0 });
  const [rawFeedsOpen, setRawFeedsOpen] = useState<string[]>(["raw"]);
  const [manualExtrinsicsEnabled, setManualExtrinsicsEnabled] = useState(false);
  const [manualExtrinsics, setManualExtrinsics] = useState({
    x: 0,
    y: 0,
    z: 0,
    roll: 0,
    pitch: 0,
    yaw: 0,
  });
  const [driveSpeed, setDriveSpeed] = useState(0.32);
  const [motionDeadlineMs, setMotionDeadlineMs] = useState(0);
  const [deadmanCountdown, setDeadmanCountdown] = useState(0);
  const [leftBank, setLeftBank] = useState(0);
  const [rightBank, setRightBank] = useState(0);
  const [jointValues, setJointValues] = useState<Record<string, number>>({});
  const [trainingTag, setTrainingTag] = useState("training");
  const [captureTargetImages, setCaptureTargetImages] = useState(120);
  const [captureExtractionFps, setCaptureExtractionFps] = useState(2.0);
  const [captureBufferSec, setCaptureBufferSec] = useState(8);
  const [captureDurationOverrideSec, setCaptureDurationOverrideSec] = useState(0);
  const [mapName, setMapName] = useState("map_snapshot");
  const [goalX, setGoalX] = useState(0);
  const [goalY, setGoalY] = useState(0);
  const [goalYaw, setGoalYaw] = useState(0);
  const [demoCount, setDemoCount] = useState(3);
  const [logSource, setLogSource] = useState("robot_console");
  const [logSeverity, setLogSeverity] = useState<SeverityFilter>("all");
  const [logQuery, setLogQuery] = useState("");
  const [logEntries, setLogEntries] = useState<LogEntry[]>([]);

  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimerRef = useRef<number | null>(null);
  const driveTimerRef = useRef<number | null>(null);
  const activeDriveRef = useRef<string>("");
  const keyboardDriveRef = useRef<string>("");
  const logEventSourceRef = useRef<EventSource | null>(null);

  const status = robot.status;
  const capabilities = status?.capabilities || {};
  const recordingStatus = status?.recording?.status;
  const capturePlan = status?.recording?.capture_plan;
  const bagStoragePath = recordingStatus?.storage_path || "n/a";
  const activeOutputDir = recordingStatus?.active?.output_dir || capturePlan?.output_dir || "";
  const extractedOutputPath = capturePlan?.extractor_output_path || "n/a";
  const captureRemainingSec =
    capturePlan?.active && capturePlan?.stop_at_unix
      ? Math.max(0, capturePlan.stop_at_unix - Date.now() / 1000)
      : 0;
  const plannedCaptureDurationSec =
    captureDurationOverrideSec > 0
      ? captureDurationOverrideSec
      : (captureTargetImages / Math.max(0.1, captureExtractionFps)) + captureBufferSec;
  const logSources: LogSource[] = status?.logs?.sources || [];
  const cameraStreamTopics = status?.visualizer?.camera?.available_topics || [];
  const mapTopic = status?.visualizer?.map?.topic || "";
  const pathTopic = status?.visualizer?.map?.selected_path_topic || "";
  const armJoints = (status?.arm?.joints || []) as ArmJointConfig[];
  const namedPoses = status?.arm?.named_poses || [];

  const motionActive = motionDeadlineMs > Date.now();
  const scanPayload = robot.scanPayload;
  const mapPayload = robot.mapPayload;
  const obstacleMapPayload = robot.obstacleMapPayload;
  const mapOverlay = robot.mapOverlay;
  const depthPayload = robot.depthPayload;
  const detectionPayload = robot.detectionPayload;

  const refreshStatus = useCallback(async () => {
    const next = await getStatus();
    setStatus(next);
  }, [setStatus]);

  const connectWebSocket = useCallback(() => {
    if (wsRef.current && wsRef.current.readyState <= WebSocket.OPEN) {
      return;
    }

    const proto = window.location.protocol === "https:" ? "wss" : "ws";
    const wsUrl = `${proto}://${window.location.host}/ws`;
    setWsStateLabel("connecting");

    const ws = new WebSocket(wsUrl);
    wsRef.current = ws;

    ws.onopen = () => {
      setWsConnected(true);
      setWsStateLabel("connected");
      setModeReconciled(false);
      setCameraReconciled(false);
    };

    ws.onmessage = (event) => {
      try {
        const payload = JSON.parse(event.data) as WsEvent;
        if (payload.type === "status") {
          setStatus(payload.data as StatusPayload);
          return;
        }
        if (payload.type === "scan") {
          setScanPayload(payload.data as ScanPayload);
          return;
        }
        if (payload.type === "map") {
          setMapPayload(payload.data as MapPayload);
          return;
        }
        if (payload.type === "obstacle_map") {
          setObstacleMapPayload(payload.data as ObstacleMapPayload);
          return;
        }
        if (payload.type === "map_overlay") {
          setMapOverlay(payload.data as MapOverlayPayload);
          return;
        }
        if (payload.type === "pointcloud") {
          setPointcloudPayload(payload.data as PointcloudPayload);
          return;
        }
        if (payload.type === "depth") {
          setDepthPayload(payload.data as DepthPayload);
          return;
        }
        if (payload.type === "detections") {
          setDetectionPayload(payload.data as DetectionPayload);
        }
      } catch {
        // ignore malformed payloads
      }
    };

    ws.onclose = () => {
      setWsConnected(false);
      setWsStateLabel("reconnecting");
      wsRef.current = null;
      if (reconnectTimerRef.current !== null) {
        window.clearTimeout(reconnectTimerRef.current);
      }
      reconnectTimerRef.current = window.setTimeout(() => connectWebSocket(), 1200);
    };

    ws.onerror = () => {
      setWsStateLabel("error");
      ws.close();
    };
  }, [
    setCameraReconciled,
    setMapOverlay,
    setMapPayload,
    setObstacleMapPayload,
    setDepthPayload,
    setDetectionPayload,
    setModeReconciled,
    setPointcloudPayload,
    setScanPayload,
    setStatus,
    setWsConnected,
    setWsStateLabel,
  ]);

  useEffect(() => {
    refreshStatus().catch((err) => messageApi.error(err.message));
    connectWebSocket();

    return () => {
      if (reconnectTimerRef.current !== null) {
        window.clearTimeout(reconnectTimerRef.current);
      }
      if (driveTimerRef.current !== null) {
        window.clearInterval(driveTimerRef.current);
      }
      if (logEventSourceRef.current) {
        logEventSourceRef.current.close();
      }
      wsRef.current?.close();
    };
  }, [connectWebSocket, messageApi, refreshStatus]);

  useEffect(() => {
    if (!modeDirty) {
      setModeDraft(robot.selectedMode);
    }
    if (modeDraft === robot.selectedMode) {
      setModeDirty(false);
    }
  }, [modeDirty, modeDraft, robot.selectedMode]);

  useEffect(() => {
    if (!cameraDraft && robot.selectedCameraTopic) {
      setCameraDraft(robot.selectedCameraTopic);
    }
  }, [cameraDraft, robot.selectedCameraTopic]);

  useEffect(() => {
    if (!mapTopicDraft && mapTopic) {
      setMapTopicDraft(mapTopic);
    }
  }, [mapTopic, mapTopicDraft]);

  useEffect(() => {
    if (!pathTopicDraft && pathTopic) {
      setPathTopicDraft(pathTopic);
    }
  }, [pathTopic, pathTopicDraft]);

  useEffect(() => {
    if (costmapTopicDraft || !status?.visualizer?.obstacle_map?.selected_topic) {
      return;
    }
    setCostmapTopicDraft(status.visualizer.obstacle_map.selected_topic);
  }, [costmapTopicDraft, status?.visualizer?.obstacle_map?.selected_topic]);

  useEffect(() => {
    if (!pointcloudDraft && status?.visualizer?.pointcloud?.selected_topic) {
      setPointcloudDraft(status.visualizer.pointcloud.selected_topic);
    }
  }, [pointcloudDraft, status?.visualizer?.pointcloud?.selected_topic]);

  useEffect(() => {
    if (scanTopicDraft || !status?.visualizer?.scan?.topic) {
      return;
    }
    setScanTopicDraft(status.visualizer.scan.topic);
  }, [scanTopicDraft, status?.visualizer?.scan?.topic]);

  useEffect(() => {
    if (rgbTopicDraft || !status?.visualizer?.rgb?.topic) {
      return;
    }
    setRgbTopicDraft(status.visualizer.rgb.topic);
  }, [rgbTopicDraft, status?.visualizer?.rgb?.topic]);

  useEffect(() => {
    if (depthTopicDraft || !status?.visualizer?.depth?.topic) {
      return;
    }
    setDepthTopicDraft(status.visualizer.depth.topic);
  }, [depthTopicDraft, status?.visualizer?.depth?.topic]);

  useEffect(() => {
    if (detectionTopicDraft || !status?.visualizer?.detections?.topic) {
      return;
    }
    setDetectionTopicDraft(status.visualizer.detections.topic);
  }, [detectionTopicDraft, status?.visualizer?.detections?.topic]);

  useEffect(() => {
    if (fixedFrameDraft !== "auto") {
      return;
    }
    const selected = status?.visualizer?.tf?.fixed_frame?.selected;
    if (!selected) {
      return;
    }
    setFixedFrameDraft(selected);
  }, [fixedFrameDraft, status?.visualizer?.tf?.fixed_frame?.selected]);

  useEffect(() => {
    try {
      const raw = window.localStorage.getItem(VIS_SETTINGS_KEY);
      if (!raw) {
        return;
      }
      const parsed = JSON.parse(raw) as {
        fixedFrame?: string;
        mainFollowMode?: boolean;
        showTfChainOverlay?: boolean;
        showAxesOverlay?: boolean;
        topics?: {
          scan?: string;
          rgb?: string;
          depth?: string;
          detections?: string;
          map?: string;
          costmap?: string;
          path?: string;
          pointcloud?: string;
          camera_stream?: string;
        };
        manualExtrinsicsEnabled?: boolean;
        manualExtrinsics?: {
          x?: number;
          y?: number;
          z?: number;
          roll?: number;
          pitch?: number;
          yaw?: number;
        };
      };
      if (parsed.fixedFrame) {
        setFixedFrameDraft(parsed.fixedFrame);
      }
      if (parsed.topics?.scan) {
        setScanTopicDraft(parsed.topics.scan);
      }
      if (parsed.topics?.rgb) {
        setRgbTopicDraft(parsed.topics.rgb);
      }
      if (parsed.topics?.depth) {
        setDepthTopicDraft(parsed.topics.depth);
      }
      if (parsed.topics?.detections) {
        setDetectionTopicDraft(parsed.topics.detections);
      }
      if (parsed.topics?.map) {
        setMapTopicDraft(parsed.topics.map);
      }
      if (parsed.topics?.costmap) {
        setCostmapTopicDraft(parsed.topics.costmap);
      }
      if (parsed.topics?.path) {
        setPathTopicDraft(parsed.topics.path);
      }
      if (parsed.topics?.pointcloud) {
        setPointcloudDraft(parsed.topics.pointcloud);
      }
      if (parsed.topics?.camera_stream) {
        setCameraDraft(parsed.topics.camera_stream);
      }
      if (typeof parsed.manualExtrinsicsEnabled === "boolean") {
        setManualExtrinsicsEnabled(parsed.manualExtrinsicsEnabled);
      }
      if (typeof parsed.mainFollowMode === "boolean") {
        setMainFollowMode(parsed.mainFollowMode);
      }
      if (typeof parsed.showTfChainOverlay === "boolean") {
        setShowTfChainOverlay(parsed.showTfChainOverlay);
      }
      if (typeof parsed.showAxesOverlay === "boolean") {
        setShowAxesOverlay(parsed.showAxesOverlay);
      }
      if (parsed.manualExtrinsics) {
        setManualExtrinsics((prev) => ({
          x: Number(parsed.manualExtrinsics?.x ?? prev.x),
          y: Number(parsed.manualExtrinsics?.y ?? prev.y),
          z: Number(parsed.manualExtrinsics?.z ?? prev.z),
          roll: Number(parsed.manualExtrinsics?.roll ?? prev.roll),
          pitch: Number(parsed.manualExtrinsics?.pitch ?? prev.pitch),
          yaw: Number(parsed.manualExtrinsics?.yaw ?? prev.yaw),
        }));
      }
    } catch {
      // ignore storage parse issues
    }
  }, []);

  useEffect(() => {
    const payload = {
      fixedFrame: fixedFrameDraft,
      topics: {
        scan: scanTopicDraft,
        rgb: rgbTopicDraft,
        depth: depthTopicDraft,
        detections: detectionTopicDraft,
        map: mapTopicDraft,
        costmap: costmapTopicDraft,
        path: pathTopicDraft,
        pointcloud: pointcloudDraft,
        camera_stream: cameraDraft,
      },
      mainFollowMode,
      showTfChainOverlay,
      showAxesOverlay,
      manualExtrinsicsEnabled,
      manualExtrinsics,
    };
    try {
      window.localStorage.setItem(VIS_SETTINGS_KEY, JSON.stringify(payload));
    } catch {
      // ignore storage write issues
    }
  }, [
    cameraDraft,
    depthTopicDraft,
    detectionTopicDraft,
    fixedFrameDraft,
    mainFollowMode,
    manualExtrinsics,
    manualExtrinsicsEnabled,
    mapTopicDraft,
    costmapTopicDraft,
    pathTopicDraft,
    pointcloudDraft,
    rgbTopicDraft,
    scanTopicDraft,
    showAxesOverlay,
    showTfChainOverlay,
  ]);

  useEffect(() => {
    const hasNativePath =
      status?.visualizer?.tf?.has_base_laser_tf && status?.visualizer?.tf?.has_base_oak_tf;
    if (!hasNativePath || !manualExtrinsicsEnabled) {
      return;
    }
    setManualExtrinsicsEnabled(false);
    messageApi.info("Manual extrinsics disabled because TF path base_link->laser/oak is now available.");
  }, [
    manualExtrinsicsEnabled,
    messageApi,
    status?.visualizer?.tf?.has_base_laser_tf,
    status?.visualizer?.tf?.has_base_oak_tf,
  ]);

  useEffect(() => {
    const timer = window.setInterval(() => {
      const remaining = Math.max(0, motionDeadlineMs - Date.now());
      setDeadmanCountdown(remaining / 1000);
      if (remaining <= 0 && motionDeadlineMs > 0) {
        setMotionDeadlineMs(0);
        postApi("/api/arm", { armed: false }).catch(() => undefined);
      }
    }, 150);

    return () => window.clearInterval(timer);
  }, [motionDeadlineMs]);

  useEffect(() => {
    if (!status || robot.modeReconciled) {
      return;
    }

    const cached = getCachedMode();
    if (cached && cached !== status.mode) {
      postApi("/api/mode", { mode: cached })
        .then(() => {
          setSelectedMode(cached);
          setModeReconciled(true);
          messageApi.success(`Reconciled mode to ${cached}`);
        })
        .catch(() => {
          setModeReconciled(true);
        });
      return;
    }

    setModeReconciled(true);
  }, [messageApi, robot.modeReconciled, setModeReconciled, setSelectedMode, status]);

  useEffect(() => {
    if (!status || cameraReconciled) {
      return;
    }
    const cached = getCachedCameraTopic();
    const available = status.visualizer?.camera?.available_topics || [];
    if (cached && cached !== status.visualizer?.camera?.selected_topic && available.includes(cached)) {
      postApi("/api/camera/select", { topic: cached })
        .then(() => {
          setSelectedCameraTopic(cached);
          setCameraReconciled(true);
        })
        .catch(() => setCameraReconciled(true));
      return;
    }
    setCameraReconciled(true);
  }, [cameraReconciled, setCameraReconciled, setSelectedCameraTopic, status]);

  useEffect(() => {
    if (!armJoints.length) {
      return;
    }
    setJointValues((prev) => {
      const next = { ...prev };
      for (const joint of armJoints) {
        if (!(joint.name in next)) {
          next[joint.name] = Number(joint.default ?? 0);
        }
      }
      return next;
    });
  }, [armJoints]);

  useEffect(() => {
    if (ui.navKey !== "logs") {
      if (logEventSourceRef.current) {
        logEventSourceRef.current.close();
        logEventSourceRef.current = null;
      }
      return;
    }

    if (logEventSourceRef.current) {
      logEventSourceRef.current.close();
    }

    const source = encodeURIComponent(logSource);
    const es = new EventSource(`/api/logs/stream?source=${source}&tail=120`);
    logEventSourceRef.current = es;

    es.onmessage = (evt) => {
      try {
        const payload = JSON.parse(evt.data) as { ts?: number; source?: string; line?: string; timestamp_unix?: number };
        const line = String(payload.line || "");
        const sourceId = String(payload.source || logSource);
        const ts = Number(payload.timestamp_unix || Date.now() / 1000);

        setLogEntries((prev) => {
          const next = [...prev, { ts, source: sourceId, line, severity: classifySeverity(line) }];
          if (next.length > 800) {
            return next.slice(next.length - 800);
          }
          return next;
        });
      } catch {
        // ignore malformed log events
      }
    };

    es.onerror = () => {
      es.close();
      logEventSourceRef.current = null;
    };

    return () => {
      es.close();
      if (logEventSourceRef.current === es) {
        logEventSourceRef.current = null;
      }
    };
  }, [logSource, ui.navKey]);

  const sendBase = useCallback(
    async (payload: Record<string, unknown>) => {
      await postApi("/api/manual/base", payload);
    },
    [],
  );

  const startDrive = useCallback(
    async (direction: string) => {
      if (!motionActive) {
        messageApi.warning("Enable motion first (deadman)");
        return;
      }

      const vector = DRIVE_VECTORS[direction];
      if (!vector) {
        return;
      }

      if (driveTimerRef.current !== null) {
        window.clearInterval(driveTimerRef.current);
      }
      activeDriveRef.current = direction;

      const tick = async () => {
        try {
          await sendBase({
            type: "cmd_vel",
            linear_x: vector.linear_x * driveSpeed,
            angular_z: vector.angular_z * Math.max(0.4, driveSpeed),
          });
        } catch {
          // keep trying while held
        }
      };

      await tick();
      driveTimerRef.current = window.setInterval(() => {
        tick().catch(() => undefined);
      }, 120);
    },
    [driveSpeed, messageApi, motionActive, sendBase],
  );

  const stopDrive = useCallback(() => {
    if (driveTimerRef.current !== null) {
      window.clearInterval(driveTimerRef.current);
      driveTimerRef.current = null;
    }
    activeDriveRef.current = "";
    sendBase({ type: "stop" }).catch(() => undefined);
  }, [sendBase]);

  useEffect(() => {
    const onKeyDown = (event: KeyboardEvent) => {
      if (ui.navKey !== "manual" || isEditableElement(event.target)) {
        return;
      }
      const map: Record<string, string> = {
        w: "forward",
        a: "left",
        s: "backward",
        d: "right"
      };
      const drive = map[event.key.toLowerCase()];
      if (!drive || keyboardDriveRef.current === drive) {
        return;
      }
      keyboardDriveRef.current = drive;
      startDrive(drive);
    };

    const onKeyUp = (event: KeyboardEvent) => {
      const map: Record<string, string> = {
        w: "forward",
        a: "left",
        s: "backward",
        d: "right"
      };
      const drive = map[event.key.toLowerCase()];
      if (!drive) {
        return;
      }
      if (keyboardDriveRef.current === drive) {
        keyboardDriveRef.current = "";
      }
      stopDrive();
    };

    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);
    return () => {
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
    };
  }, [startDrive, stopDrive, ui.navKey]);

  const armDeadman = useCallback(async () => {
    await postApi("/api/arm", { armed: true });
    setMotionDeadlineMs(Date.now() + 5000);
    messageApi.success("Motion enabled for 5 seconds");
  }, [messageApi]);

  const applyMode = useCallback(async () => {
    await postApi("/api/mode", { mode: modeDraft });
    setSelectedMode(modeDraft);
    setModeDirty(false);
    await refreshStatus();
    messageApi.success(`Mode set to ${modeDraft}`);
  }, [messageApi, modeDraft, refreshStatus, setSelectedMode]);

  const applyModeImmediate = useCallback(
    async (nextMode: RobotMode) => {
      try {
        await postApi("/api/mode", { mode: nextMode });
        setSelectedMode(nextMode);
        setModeDraft(nextMode);
        setModeDirty(false);
        await refreshStatus();
      } catch (err) {
        const msg = err instanceof Error ? err.message : "Failed to set mode";
        messageApi.error(msg);
      }
    },
    [messageApi, refreshStatus, setSelectedMode],
  );

  const disarmUnified = useCallback(async () => {
    try {
      await postApi("/api/arm", { armed: false });
      await postApi("/api/manual/base", { type: "stop" });
      await refreshStatus();
      messageApi.success("Disarmed");
    } catch (err) {
      const msg = err instanceof Error ? err.message : "Failed to disarm";
      messageApi.error(msg);
    }
  }, [messageApi, refreshStatus]);

  const applyCameraTopic = useCallback(async () => {
    if (!cameraDraft) {
      return;
    }
    await postApi("/api/camera/select", { topic: cameraDraft });
    setSelectedCameraTopic(cameraDraft);
    await refreshStatus();
  }, [cameraDraft, refreshStatus, setSelectedCameraTopic]);

  const applyVisualizerConfig = useCallback(async () => {
    await postApi("/api/visualizer/config", {
      scan_topic: scanTopicDraft || undefined,
      rgb_topic: rgbTopicDraft || undefined,
      depth_topic: depthTopicDraft || undefined,
      detection_topic: detectionTopicDraft || undefined,
      map_topic: mapTopicDraft || undefined,
      costmap_topic: costmapTopicDraft || undefined,
      path_topic: pathTopicDraft || undefined,
      pointcloud_topic: pointcloudDraft || undefined,
      fixed_frame: fixedFrameDraft === "auto" ? "" : fixedFrameDraft,
      camera_stream_topic: cameraDraft || undefined,
    });
    await refreshStatus();
  }, [
    cameraDraft,
    depthTopicDraft,
    detectionTopicDraft,
    fixedFrameDraft,
    mapTopicDraft,
    costmapTopicDraft,
    pathTopicDraft,
    pointcloudDraft,
    refreshStatus,
    rgbTopicDraft,
    scanTopicDraft,
  ]);

  const sendMotorBank = useCallback(async () => {
    await sendBase({ type: "motor_bank", left: leftBank, right: rightBank });
  }, [leftBank, rightBank, sendBase]);

  const sendArmJoints = useCallback(async () => {
    await postApi("/api/manual/arm", { joints: jointValues });
  }, [jointValues]);

  const runAction = useCallback(
    async (label: string, fn: () => Promise<unknown>) => {
      try {
        await fn();
        await refreshStatus();
        messageApi.success(label);
      } catch (err) {
        const msg = err instanceof Error ? err.message : "Action failed";
        messageApi.error(msg);
      }
    },
    [messageApi, refreshStatus],
  );

  const filteredLogs = useMemo(() => {
    return logEntries.filter((entry) => {
      if (logSeverity !== "all" && entry.severity !== logSeverity) {
        return false;
      }
      if (!logQuery.trim()) {
        return true;
      }
      return entry.line.toLowerCase().includes(logQuery.toLowerCase());
    });
  }, [logEntries, logQuery, logSeverity]);

  const recordingTableData = useMemo(
    () => (status?.recording?.recent || []).map((item) => ({ ...item, key: item.path })),
    [status?.recording?.recent],
  );

  const copyDebugSnapshot = useCallback(async () => {
    const payload = {
      selected_topics: {
        scan: scanTopicDraft || status?.visualizer?.scan?.topic,
        rgb: rgbTopicDraft || status?.visualizer?.rgb?.topic,
        depth: depthTopicDraft || status?.visualizer?.depth?.topic,
        detections: detectionTopicDraft || status?.visualizer?.detections?.topic,
        map: mapTopicDraft || status?.visualizer?.map?.topic,
        costmap: costmapTopicDraft || status?.visualizer?.obstacle_map?.selected_topic,
        plan: pathTopicDraft || status?.visualizer?.map?.selected_path_topic,
        pointcloud: pointcloudDraft || status?.visualizer?.pointcloud?.selected_topic,
      },
      fixed_frame: {
        selected: fixedFrameDraft,
        resolved: status?.visualizer?.tf?.fixed_frame?.resolved,
      },
      missing_tf_pairs: status?.visualizer?.tf?.missing_pairs || [],
      message_age_sec: {
        scan: status?.visualizer?.scan?.age_sec,
        rgb: status?.visualizer?.rgb?.age_sec,
        depth: status?.visualizer?.depth?.age_sec,
        detections: status?.visualizer?.detections?.age_sec,
        map: status?.visualizer?.map?.age_sec,
        obstacle_map: status?.visualizer?.obstacle_map?.age_sec,
        odom: status?.health?.odom_age_sec,
      },
      obstacle_map: {
        source: status?.visualizer?.obstacle_map?.source,
        frame_id: status?.visualizer?.obstacle_map?.frame_id,
        width: status?.visualizer?.obstacle_map?.width,
        height: status?.visualizer?.obstacle_map?.height,
        resolution: status?.visualizer?.obstacle_map?.resolution,
        depth_fusion_enabled: status?.visualizer?.obstacle_map?.depth_fusion_enabled,
        warnings: status?.visualizer?.obstacle_map?.warnings || [],
      },
      view: {
        follow: mainFollowMode,
        show_tf_chain_overlay: showTfChainOverlay,
        show_axes_overlay: showAxesOverlay,
        layers: {
          map: showMapLayer,
          scan: showScanLayer,
          plan: showPlanLayer,
          detections: showDetections,
          depth: showDepthBlend,
        },
      },
      manual_extrinsics: {
        enabled: manualExtrinsicsEnabled,
        ...manualExtrinsics,
      },
    };
    await navigator.clipboard.writeText(JSON.stringify(payload, null, 2));
    messageApi.success("Debug snapshot copied");
  }, [
    depthTopicDraft,
    detectionTopicDraft,
    fixedFrameDraft,
    costmapTopicDraft,
    mainFollowMode,
    manualExtrinsics,
    manualExtrinsicsEnabled,
    mapTopicDraft,
    messageApi,
    pathTopicDraft,
    pointcloudDraft,
    rgbTopicDraft,
    scanTopicDraft,
    showAxesOverlay,
    showDepthBlend,
    showDetections,
    showMapLayer,
    showPlanLayer,
    showScanLayer,
    showTfChainOverlay,
    status?.health?.odom_age_sec,
    status?.visualizer?.depth?.age_sec,
    status?.visualizer?.detections?.age_sec,
    status?.visualizer?.detections?.topic,
    status?.visualizer?.map?.age_sec,
    status?.visualizer?.map?.selected_path_topic,
    status?.visualizer?.map?.topic,
    status?.visualizer?.obstacle_map?.age_sec,
    status?.visualizer?.obstacle_map?.depth_fusion_enabled,
    status?.visualizer?.obstacle_map?.frame_id,
    status?.visualizer?.obstacle_map?.height,
    status?.visualizer?.obstacle_map?.resolution,
    status?.visualizer?.obstacle_map?.selected_topic,
    status?.visualizer?.obstacle_map?.source,
    status?.visualizer?.obstacle_map?.warnings,
    status?.visualizer?.obstacle_map?.width,
    status?.visualizer?.pointcloud?.selected_topic,
    status?.visualizer?.rgb?.age_sec,
    status?.visualizer?.rgb?.topic,
    status?.visualizer?.scan?.age_sec,
    status?.visualizer?.scan?.topic,
    status?.visualizer?.tf?.fixed_frame?.resolved,
    status?.visualizer?.tf?.missing_pairs,
  ]);

  const baseMenuItems = [
    { key: "operations", icon: <DashboardOutlined />, label: "Operations" },
    { key: "visualizer", icon: <RadarChartOutlined />, label: "Visualizer" },
    { key: "manual", icon: <ControlOutlined />, label: "Manual Base" },
    { key: "unified", icon: <ControlOutlined />, label: "Unified Ctrl" },
    { key: "arm", icon: <RobotOutlined />, label: "Arm Control" },
    { key: "training", icon: <ExperimentOutlined />, label: "Training/Data" },
    { key: "demo", icon: <ToolOutlined />, label: "Demo Runs" },
    { key: "logs", icon: <ApiOutlined />, label: "Logs" },
  ];

  const severityTag = (severity: SeverityFilter): JSX.Element => {
    const color = severity === "error" ? "error" : severity === "warn" ? "warning" : "processing";
    return <Tag color={color}>{severity.toUpperCase()}</Tag>;
  };

  const renderOperations = (): JSX.Element => (
    <Space direction="vertical" size={16} style={{ width: "100%" }}>
      <Row gutter={[16, 16]}>
        <Col xs={24} lg={12}>
          <Card title="Safety Gate" className="hmi-card">
            <Space wrap>
              <Tag color={status?.safety?.armed ? "success" : "default"}>{status?.safety?.armed ? "ARMED" : "DISARMED"}</Tag>
              <Tag color={status?.safety?.estop_latched ? "error" : "success"}>{status?.safety?.estop_latched ? "E-STOP LATCHED" : "E-STOP CLEAR"}</Tag>
              <Tag color={status?.safety?.watchdog_ok ? "success" : "warning"}>{status?.safety?.watchdog_ok ? "WATCHDOG OK" : "WATCHDOG TRIPPED"}</Tag>
            </Space>
            <Divider />
            <Space wrap>
              <Button onClick={() => runAction("Armed", () => postApi("/api/arm", { armed: true }))} icon={<SafetyCertificateOutlined />}>Arm</Button>
              <Button onClick={() => runAction("Disarmed", () => postApi("/api/arm", { armed: false }))}>Disarm</Button>
              <Button danger onClick={() => runAction("E-stop latched", () => postApi("/api/safety/estop", {}))} icon={<WarningOutlined />}>E-Stop</Button>
              <Button onClick={() => runAction("E-stop reset", () => postApi("/api/safety/reset_estop", {}))}>Reset E-Stop</Button>
              <Button danger type="primary" onClick={() => runAction("Stop All complete", () => postApi("/api/stop_all", {}))} icon={<PoweroffOutlined />}>Stop All</Button>
            </Space>
          </Card>
        </Col>
        <Col xs={24} lg={12}>
          <Card title="Stack + Runtime" className="hmi-card">
            <Space wrap>
              <Button onClick={() => runAction("Stack start command sent", () => postApi("/api/stack/start", {}))} disabled={capabilities.stack_start === false}>Start Stack</Button>
              <Button onClick={() => runAction("Stack stop command sent", () => postApi("/api/stack/stop", {}))} disabled={capabilities.stack_stop === false}>Stop Stack</Button>
              <Button onClick={() => runAction("SLAM start requested", () => postApi("/api/mapping/start", {}))} disabled={capabilities.mapping_start === false}>Start SLAM</Button>
              <Button onClick={() => runAction("SLAM stop requested", () => postApi("/api/mapping/stop", {}))} disabled={capabilities.mapping_stop === false}>Stop SLAM</Button>
              <Button onClick={() => runAction("Localization switch requested", () => postApi("/api/mapping/switch_localization", {}))} disabled={capabilities.switch_localization === false}>Switch to Localization</Button>
            </Space>
            <Divider />
            <Space.Compact style={{ width: "100%" }}>
              <Input value={mapName} onChange={(event) => setMapName(event.target.value)} placeholder="Map name" />
              <Button onClick={() => runAction("Map save requested", () => postApi("/api/mapping/save", { filename: mapName }))} disabled={capabilities.mapping_save === false}>Save Map</Button>
            </Space.Compact>
            <Divider />
            <Space.Compact style={{ width: "100%" }}>
              <InputNumber value={goalX} onChange={(v) => setGoalX(Number(v || 0))} step={0.1} addonBefore="X" style={{ width: "33%" }} />
              <InputNumber value={goalY} onChange={(v) => setGoalY(Number(v || 0))} step={0.1} addonBefore="Y" style={{ width: "33%" }} />
              <InputNumber value={goalYaw} onChange={(v) => setGoalYaw(Number(v || 0))} step={0.1} addonBefore="Yaw" style={{ width: "34%" }} />
            </Space.Compact>
            <Space style={{ marginTop: 12 }} wrap>
              <Button onClick={() => runAction("Nav goal sent", () => postApi("/api/nav/goal", { x: goalX, y: goalY, yaw: goalYaw }))} disabled={capabilities.nav_goal === false}>Send Goal</Button>
              <Button onClick={() => runAction("Nav cancel requested", () => postApi("/api/nav/cancel", {}))} disabled={capabilities.nav_cancel === false}>Cancel Goal</Button>
              <Button onClick={() => runAction("Costmaps cleared", () => postApi("/api/nav/clear_costmaps", {}))} disabled={capabilities.clear_costmaps === false}>Clear Costmaps</Button>
            </Space>
          </Card>
        </Col>
      </Row>

      <Row gutter={[16, 16]}>
        <Col xs={24} lg={12}>
          <Card title="Recording" className="hmi-card">
            <Space.Compact style={{ width: "100%" }}>
              <Input value={trainingTag} onChange={(event) => setTrainingTag(event.target.value)} placeholder="Tag (e.g. shift-a-dataset)" />
              <Button type="primary" onClick={() => runAction("Recording started", () => postApi("/api/recording/start", { tags: trainingTag }))}>Start</Button>
              <Button onClick={() => runAction("Recording stopped", () => postApi("/api/recording/stop", {}))}>Stop</Button>
            </Space.Compact>
            <Divider />
            <Text type="secondary">Storage: {status?.recording?.status?.storage_path || "n/a"}</Text>
            <br />
            <Text type="secondary">Active bag: {status?.recording?.status?.active?.output_dir || "n/a"}</Text>
            <Table
              size="small"
              pagination={false}
              style={{ marginTop: 8 }}
              dataSource={recordingTableData}
              columns={[
                { title: "Name", dataIndex: "name", key: "name" },
                { title: "Size", dataIndex: "size_bytes", key: "size", width: 120, render: (v: number) => bytes(v) },
                { title: "Updated", dataIndex: "mtime_unix", key: "mtime", width: 180, render: (v: number) => timestampToLocal(v) },
              ]}
            />
          </Card>
        </Col>
        <Col xs={24} lg={12}>
          <Card title="System Health" className="hmi-card">
            <Row gutter={[12, 12]}>
              <Col span={12}><Statistic title="Lidar" value={status?.health?.lidar_rate_hz || 0} precision={1} suffix="Hz" /></Col>
              <Col span={12}><Statistic title="Camera" value={status?.health?.camera_stream_rate_hz || 0} precision={1} suffix="Hz" /></Col>
              <Col span={12}><Statistic title="CPU Temp" value={status?.reliability?.cpu_temp_c || 0} precision={1} suffix="°C" /></Col>
              <Col span={12}><Statistic title="Disk Free" value={bytes(status?.reliability?.disk_free_bytes || 0)} /></Col>
            </Row>
            <Divider />
            <Alert
              type={status?.health?.base_connected ? "success" : "warning"}
              showIcon
              message={status?.health?.base_connected ? "Base feedback online" : "Base feedback missing"}
              description={`Last odom update: ${secAge(status?.health?.odom_age_sec)}`}
            />
          </Card>
        </Col>
      </Row>
    </Space>
  );

  const renderVisualizer = (): JSX.Element => {
    const catalog = status?.visualizer?.topic_catalog;
    const fixedInfo = status?.visualizer?.tf?.fixed_frame;
    const pose = mapOverlay?.robot_pose || status?.visualizer?.map?.pose;
    const missingPairs = status?.visualizer?.tf?.missing_pairs || [];
    const plannerActive = status?.visualizer?.map?.planner_active || false;
    const obstacleStatus = status?.visualizer?.obstacle_map;
    const pointcloudOptions = catalog?.pointcloud?.options || [];

    const topicBadge = (options: Array<{ topic: string; publisher_count: number }>, topicName: string): JSX.Element | null => {
      const item = options.find((opt) => opt.topic === topicName);
      if (!item || item.publisher_count > 0) {
        return null;
      }
      return <Tag color="warning">No publisher</Tag>;
    };

    const detectionItems = detectionPayload?.detections || [];
    const detectionsForMinimap = showDetections ? detectionPayload : null;
    const scanPointsPresent = Boolean(
      (mapOverlay?.scan_points?.length || 0) > 0 || (mapOverlay?.scan_points_sensor?.length || 0) > 0,
    );
    const planPointsPresent = Boolean((mapOverlay?.path_points?.length || 0) > 0);
    const hasObstacleLayer = Boolean(obstacleMapPayload?.data_rle?.length);
    const hasMapLayer = Boolean(mapPayload?.data_rle?.length);

    const missingInputReasons: string[] = [];
    if (showMapLayer && !hasObstacleLayer && !hasMapLayer) {
      missingInputReasons.push("No map/costmap payload yet");
    }
    if (showScanLayer && !scanPointsPresent) {
      missingInputReasons.push("No scan points yet");
    }
    if (showPlanLayer && !planPointsPresent) {
      missingInputReasons.push("No local plan topic publishing");
    }
    if (showDetections && detectionItems.length === 0) {
      missingInputReasons.push("No detections in current frame");
    }

    return (
      <Space direction="vertical" size={16} style={{ width: "100%" }}>
        <Card className="hmi-card">
          <Row gutter={[12, 8]} align="middle">
            <Col xs={24} lg={14}>
              <Space wrap>
                <Tag color={status?.visualizer?.tf?.scan_to_fixed_ok ? "success" : "warning"}>
                  TF {status?.visualizer?.tf?.scan_to_fixed_ok ? "OK" : "MISSING"}
                </Tag>
                <Text type="secondary">/tf age {secAge(status?.visualizer?.tf?.tf_age_sec)}</Text>
                <Text type="secondary">scan {hz(status?.visualizer?.scan?.fps)}</Text>
                <Text type="secondary">rgb {hz(status?.visualizer?.rgb?.fps)}</Text>
                <Text type="secondary">depth {hz(status?.visualizer?.depth?.fps)}</Text>
                <Text type="secondary">det {hz(status?.visualizer?.detections?.fps)}</Text>
                <Text type="secondary">obs {hz(obstacleStatus?.fps)}</Text>
                <Text type="secondary">main {hz(mainRenderStats.fps)} fps</Text>
                <Text type="secondary">mini {hz(miniRenderStats.fps)} fps</Text>
                <Text type="secondary">drops m:{mainRenderStats.droppedFrames} mini:{miniRenderStats.droppedFrames}</Text>
                <Tag color={obstacleStatus?.source === "nav2" ? "blue" : "gold"}>
                  obstacle {obstacleStatus?.source || "none"}
                </Tag>
              </Space>
            </Col>
            <Col xs={24} lg={10}>
              <Space wrap style={{ justifyContent: "flex-end", width: "100%" }}>
                <Button size="small" onClick={() => runAction("Visualizer topics applied", applyVisualizerConfig)}>
                  Apply Topics
                </Button>
                <Button size="small" onClick={() => copyDebugSnapshot()}>
                  Copy Debug Snapshot
                </Button>
              </Space>
            </Col>
          </Row>
          {missingPairs.length ? (
            <Alert
              style={{ marginTop: 10 }}
              type="warning"
              showIcon
              message={`Missing TF: ${missingPairs.join(", ")}`}
              description="Rendering remains active in sensor-centric fallback mode."
            />
          ) : null}
          {fixedInfo?.warnings?.length ? (
            <Alert
              style={{ marginTop: 10 }}
              type="warning"
              showIcon
              message={fixedInfo.warnings.join(" | ")}
            />
          ) : null}
          {!status?.visualizer?.tf?.has_base_link ? (
            <Alert
              style={{ marginTop: 10 }}
              type="warning"
              showIcon
              message="No base_link frame found"
              description="Robot pose falls back to /odom (if available) or origin."
            />
          ) : null}
          {!status?.visualizer?.tf?.has_odom_base_tf ? (
            <Alert
              style={{ marginTop: 10 }}
              type="warning"
              showIcon
              message="No odom->base_link TF"
              description="Real moving map requires odom->base_link TF to be published continuously."
            />
          ) : null}
          {missingInputReasons.length ? (
            <Alert
              style={{ marginTop: 10 }}
              type="info"
              showIcon
              message="Visualizer is rendering partial data"
              description={missingInputReasons.join(" | ")}
            />
          ) : null}
        </Card>

        <Row gutter={[16, 16]}>
          <Col xs={24} xxl={17}>
            <Card className="hmi-card" title="3D / BEV Fused View">
              <Space direction="vertical" style={{ width: "100%" }} size={10}>
                <Space wrap>
                  <Select
                    style={{ minWidth: 220 }}
                    value={scanTopicDraft || undefined}
                    placeholder="LiDAR topic"
                    options={(catalog?.scan?.options || []).map((opt) => ({
                      value: opt.topic,
                      label: `${opt.topic} (${opt.publisher_count})`,
                    }))}
                    onChange={(value) => setScanTopicDraft(value)}
                  />
                  {topicBadge(catalog?.scan?.options || [], scanTopicDraft || status?.visualizer?.scan?.topic || "")}
                  <Select
                    style={{ minWidth: 220 }}
                    value={detectionTopicDraft || undefined}
                    placeholder="Detections topic"
                    options={(catalog?.detections?.options || []).map((opt) => ({
                      value: opt.topic,
                      label: `${opt.topic} (${opt.publisher_count})`,
                    }))}
                    onChange={(value) => setDetectionTopicDraft(value)}
                  />
                  {topicBadge(catalog?.detections?.options || [], detectionTopicDraft || status?.visualizer?.detections?.topic || "")}
                  <Select
                    style={{ minWidth: 160 }}
                    value={fixedFrameDraft || "auto"}
                    options={[
                      { value: "auto", label: "Fixed: Auto" },
                      { value: "map", label: "Fixed: map" },
                      { value: "odom", label: "Fixed: odom" },
                      { value: "base_link", label: "Fixed: base_link" },
                    ]}
                    onChange={(value) => setFixedFrameDraft(value)}
                  />
                  <Segmented
                    options={[
                      { value: "follow", label: "Follow" },
                      { value: "free", label: "Free Pan" },
                    ]}
                    value={mainFollowMode ? "follow" : "free"}
                    onChange={(value) => setMainFollowMode(value === "follow")}
                  />
                  <Button
                    onClick={() => {
                      setMainFollowMode(true);
                      setFusedRecenterToken((prev) => prev + 1);
                    }}
                  >
                    Recenter
                  </Button>
                  <Switch checked={showMapLayer} onChange={setShowMapLayer} checkedChildren="Map" unCheckedChildren="Map" />
                  <Switch checked={showScanLayer} onChange={setShowScanLayer} checkedChildren="Scan" unCheckedChildren="Scan" />
                  <Switch checked={showPlanLayer} onChange={setShowPlanLayer} checkedChildren="Plan" unCheckedChildren="Plan" />
                  <Switch checked={showDetections} onChange={setShowDetections} checkedChildren="Detections" unCheckedChildren="Detections" />
                  <Switch checked={showDepthBlend} onChange={setShowDepthBlend} checkedChildren="Depth" unCheckedChildren="Depth" />
                  <Switch checked={showTfChainOverlay} onChange={setShowTfChainOverlay} checkedChildren="TF chain" unCheckedChildren="TF chain" />
                  <Switch checked={showAxesOverlay} onChange={setShowAxesOverlay} checkedChildren="Axes" unCheckedChildren="Axes" />
                </Space>
                <FusedViewCanvas
                  map={mapPayload}
                  obstacleMap={showMapLayer ? obstacleMapPayload : null}
                  overlay={mapOverlay}
                  detections={showDetections ? detectionPayload : null}
                  followPose={mainFollowMode}
                  panEnabled={!mainFollowMode}
                  recenterSignal={fusedRecenterToken}
                  onPanStart={() => setMainFollowMode(false)}
                  showTfOverlay={showTfChainOverlay}
                  showAxesOverlay={showAxesOverlay}
                  tfChain={{
                    fixed_frame: fixedInfo?.resolved || "n/a",
                    selected_fixed_frame: fixedInfo?.selected || "auto",
                    scan_to_fixed_ok: Boolean(status?.visualizer?.tf?.scan_to_fixed_ok),
                    detections_to_fixed_ok: Boolean(status?.visualizer?.tf?.detections_to_fixed_ok),
                    has_odom_base_tf: Boolean(status?.visualizer?.tf?.has_odom_base_tf),
                    has_base_laser_tf: Boolean(status?.visualizer?.tf?.has_base_laser_tf),
                    has_base_oak_tf: Boolean(status?.visualizer?.tf?.has_base_oak_tf),
                    missing_pairs: missingPairs,
                  }}
                  onRenderStats={(stats) =>
                    setMainRenderStats({
                      fps: stats.fps,
                      droppedFrames: stats.droppedFrames,
                    })
                  }
                  layers={{
                    map: showMapLayer,
                    obstacle: showMapLayer,
                    scan: showScanLayer,
                    detections: showDetections,
                    plan: showPlanLayer,
                    drivable: true,
                  }}
                  manualExtrinsics={{
                    enabled: manualExtrinsicsEnabled,
                    x: manualExtrinsics.x,
                    y: manualExtrinsics.y,
                    yaw: manualExtrinsics.yaw,
                  }}
                />
                <Row gutter={[12, 8]}>
                  <Col xs={24} md={12}>
                    <Text type="secondary">
                      Fixed frame: {fixedInfo?.resolved || "n/a"} (selected {fixedInfo?.selected || "auto"})
                    </Text>
                  </Col>
                  <Col xs={24} md={12}>
                    <Text type="secondary">
                      Pose:{" "}
                      {pose ? `${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}, yaw ${pose.yaw.toFixed(2)} (${pose.source || "tf"})` : "unavailable"}
                    </Text>
                  </Col>
                  <Col xs={24} md={12}>
                    <Text type="secondary">
                      Detections: {detectionItems.length} | frame {detectionPayload?.frame_id || "n/a"}
                    </Text>
                  </Col>
                  <Col xs={24} md={12}>
                    <Text type="secondary">
                      Planner: {plannerActive ? "active" : "No planner active"} | obs {obstacleStatus?.source || "none"}
                    </Text>
                  </Col>
                </Row>
              </Space>
            </Card>
          </Col>

          <Col xs={24} xxl={7}>
            <Space direction="vertical" size={16} style={{ width: "100%" }}>
              <Card className="hmi-card" title="Top-Down Minimap">
                <Space direction="vertical" style={{ width: "100%" }} size={8}>
                  <FusedViewCanvas
                    map={mapPayload}
                    obstacleMap={showMapLayer ? obstacleMapPayload : null}
                    overlay={mapOverlay}
                    detections={detectionsForMinimap}
                    width={420}
                    height={320}
                    followPose
                    panEnabled={false}
                    showAxesOverlay={showAxesOverlay}
                    onRenderStats={(stats) =>
                      setMiniRenderStats({
                        fps: stats.fps,
                        droppedFrames: stats.droppedFrames,
                      })
                    }
                    layers={{
                      map: showMapLayer,
                      obstacle: showMapLayer,
                      scan: showScanLayer,
                      detections: showDetections,
                      plan: showPlanLayer,
                      drivable: true,
                    }}
                    manualExtrinsics={{
                      enabled: manualExtrinsicsEnabled,
                      x: manualExtrinsics.x,
                      y: manualExtrinsics.y,
                      yaw: manualExtrinsics.yaw,
                    }}
                  />
                  <Space wrap>
                    <Switch size="small" checked={showScanLayer} onChange={setShowScanLayer} checkedChildren="Scan" unCheckedChildren="Scan" />
                    <Switch size="small" checked={showDepthBlend} onChange={setShowDepthBlend} checkedChildren="Depth" unCheckedChildren="Depth" />
                    <Switch size="small" checked={showDetections} onChange={setShowDetections} checkedChildren="Boxes" unCheckedChildren="Boxes" />
                    <Switch size="small" checked={showPlanLayer} onChange={setShowPlanLayer} checkedChildren="Plan" unCheckedChildren="Plan" />
                    <Switch size="small" checked={showMapLayer} onChange={setShowMapLayer} checkedChildren="Map" unCheckedChildren="Map" />
                  </Space>
                  <Space wrap>
                    <Tag color="green">Local plan</Tag>
                    <Tag color="orange">Drivable area</Tag>
                    <Tag color="cyan">LiDAR returns</Tag>
                    <Tag color="magenta">Detections</Tag>
                  </Space>
                  <Text type="secondary">
                    Pose readout: {pose ? `${pose.x.toFixed(2)} m, ${pose.y.toFixed(2)} m, ${pose.yaw.toFixed(2)} rad` : `${fixedInfo?.resolved || "frame"} | pose unavailable`}
                  </Text>
                </Space>
              </Card>

              <Card className="hmi-card" title="Topic Sources">
                <Space direction="vertical" size={8} style={{ width: "100%" }}>
                  <Select
                    value={rgbTopicDraft || undefined}
                    placeholder="RGB topic"
                    options={(catalog?.rgb?.options || []).map((opt) => ({
                      value: opt.topic,
                      label: `${opt.topic} (${opt.publisher_count})`,
                    }))}
                    onChange={(value) => setRgbTopicDraft(value)}
                  />
                  <Select
                    value={depthTopicDraft || undefined}
                    placeholder="Depth topic"
                    options={(catalog?.depth?.options || []).map((opt) => ({
                      value: opt.topic,
                      label: `${opt.topic} (${opt.publisher_count})`,
                    }))}
                    onChange={(value) => setDepthTopicDraft(value)}
                  />
                  <Select
                    value={mapTopicDraft || undefined}
                    placeholder="Map topic"
                    options={(catalog?.map?.options || []).map((opt) => ({
                      value: opt.topic,
                      label: `${opt.topic} (${opt.publisher_count})`,
                    }))}
                    onChange={(value) => setMapTopicDraft(value)}
                  />
                  <Select
                    value={costmapTopicDraft || undefined}
                    placeholder="Costmap topic"
                    options={(catalog?.costmap?.options || []).map((opt) => ({
                      value: opt.topic,
                      label: `${opt.topic} (${opt.publisher_count})`,
                    }))}
                    onChange={(value) => setCostmapTopicDraft(value)}
                  />
                  <Select
                    value={pathTopicDraft || undefined}
                    placeholder="Plan topic"
                    options={(catalog?.plans?.options || []).map((opt) => ({
                      value: opt.topic,
                      label: `${opt.topic} (${opt.publisher_count})`,
                    }))}
                    onChange={(value) => setPathTopicDraft(value)}
                  />
                  <Select
                    value={pointcloudDraft || undefined}
                    placeholder="PointCloud topic"
                    options={pointcloudOptions.map((opt) => ({
                      value: opt.topic,
                      label: `${opt.topic} (${opt.publisher_count})`,
                    }))}
                    onChange={(value) => setPointcloudDraft(value)}
                  />
                </Space>
              </Card>
            </Space>
          </Col>

          <Col xs={24}>
            <Card
              className="hmi-card"
              title="OAK-D RGB + Overlays"
              extra={
                <Space>
                  <Switch checked={showDetections} onChange={setShowDetections} checkedChildren="Detections" unCheckedChildren="Detections" />
                  <Switch checked={showDepthBlend} onChange={setShowDepthBlend} checkedChildren="Depth blend" unCheckedChildren="Depth blend" />
                </Space>
              }
            >
              <div className="camera-frame camera-overlay-frame">
                <img
                  src={`/stream/camera.mjpeg?topic=${encodeURIComponent(
                    cameraDraft || robot.selectedCameraTopic || "",
                  )}`}
                  alt="camera"
                />
                {showDepthBlend ? (
                  <DepthHeatmapCanvas depth={depthPayload} className="camera-overlay-canvas camera-depth-overlay" />
                ) : null}
                {showDetections ? (
                  <CameraDetectionOverlay detections={detectionPayload} className="camera-overlay-canvas camera-detection-overlay" />
                ) : null}
              </div>
              <Row gutter={[12, 8]} style={{ marginTop: 10 }}>
                <Col xs={24} md={12}>
                  <Text type="secondary">
                    RGB: {status?.visualizer?.rgb?.topic || "n/a"} | {hz(status?.visualizer?.rgb?.fps)} | age {secAge(status?.visualizer?.rgb?.age_sec)}
                  </Text>
                </Col>
                <Col xs={24} md={12}>
                  <Text type="secondary">
                    Depth: {status?.visualizer?.depth?.topic || "n/a"} | {hz(status?.visualizer?.depth?.fps)} | age {secAge(status?.visualizer?.depth?.age_sec)}
                  </Text>
                </Col>
                <Col xs={24} md={12}>
                  <Text type="secondary">
                    Detections: {status?.visualizer?.detections?.topic || "n/a"} | {hz(status?.visualizer?.detections?.fps)} | age {secAge(status?.visualizer?.detections?.age_sec)}
                  </Text>
                </Col>
                <Col xs={24} md={12}>
                  <Text type="secondary">
                    Camera info age: {secAge(status?.visualizer?.rgb?.camera_info_age_sec)}
                  </Text>
                </Col>
              </Row>
              <Divider />
              <Space wrap>
                <Select
                  style={{ minWidth: 320 }}
                  value={cameraDraft || undefined}
                  placeholder="Camera MJPEG topic"
                  options={cameraStreamTopics.map((topic) => ({ value: topic, label: topic }))}
                  onChange={(value) => setCameraDraft(value)}
                />
                <Button onClick={() => runAction("Camera topic selected", applyCameraTopic)}>Apply Camera Stream</Button>
                <Button
                  icon={<CameraOutlined />}
                  href={`https://studio.foxglove.dev/?ds=foxglove-websocket&ds.url=${encodeURIComponent(
                    `ws://${window.location.hostname}:8765`,
                  )}`}
                  target="_blank"
                >
                  Open Foxglove (ws://{window.location.hostname}:8765)
                </Button>
              </Space>
            </Card>
          </Col>
        </Row>

        <Card className="hmi-card" title="Manual Extrinsics (TEMP/DEBUG)">
          <Space direction="vertical" style={{ width: "100%" }}>
            <Alert
              type={manualExtrinsicsEnabled ? "warning" : "info"}
              showIcon
              message="laser -> oak-d-base-frame (temporary UI transform)"
              description={
                status?.visualizer?.tf?.has_base_laser_tf && status?.visualizer?.tf?.has_base_oak_tf
                  ? "Native TF is available. Keep this disabled."
                  : "Use only while TF chain is missing. This is auto-disabled once real TF path exists."
              }
            />
            <Space wrap>
              <Switch checked={manualExtrinsicsEnabled} onChange={setManualExtrinsicsEnabled} />
              <Text>Enable manual extrinsics</Text>
            </Space>
            <Row gutter={[8, 8]}>
              <Col span={8}>
                <InputNumber
                  style={{ width: "100%" }}
                  addonBefore="x"
                  value={manualExtrinsics.x}
                  step={0.01}
                  onChange={(v) => setManualExtrinsics((prev) => ({ ...prev, x: Number(v || 0) }))}
                />
              </Col>
              <Col span={8}>
                <InputNumber
                  style={{ width: "100%" }}
                  addonBefore="y"
                  value={manualExtrinsics.y}
                  step={0.01}
                  onChange={(v) => setManualExtrinsics((prev) => ({ ...prev, y: Number(v || 0) }))}
                />
              </Col>
              <Col span={8}>
                <InputNumber
                  style={{ width: "100%" }}
                  addonBefore="z"
                  value={manualExtrinsics.z}
                  step={0.01}
                  onChange={(v) => setManualExtrinsics((prev) => ({ ...prev, z: Number(v || 0) }))}
                />
              </Col>
              <Col span={8}>
                <InputNumber
                  style={{ width: "100%" }}
                  addonBefore="roll"
                  value={manualExtrinsics.roll}
                  step={0.01}
                  onChange={(v) => setManualExtrinsics((prev) => ({ ...prev, roll: Number(v || 0) }))}
                />
              </Col>
              <Col span={8}>
                <InputNumber
                  style={{ width: "100%" }}
                  addonBefore="pitch"
                  value={manualExtrinsics.pitch}
                  step={0.01}
                  onChange={(v) => setManualExtrinsics((prev) => ({ ...prev, pitch: Number(v || 0) }))}
                />
              </Col>
              <Col span={8}>
                <InputNumber
                  style={{ width: "100%" }}
                  addonBefore="yaw"
                  value={manualExtrinsics.yaw}
                  step={0.01}
                  onChange={(v) => setManualExtrinsics((prev) => ({ ...prev, yaw: Number(v || 0) }))}
                />
              </Col>
            </Row>
          </Space>
        </Card>

        <Collapse
          activeKey={rawFeedsOpen}
          onChange={(keys) => setRawFeedsOpen(Array.isArray(keys) ? keys : [String(keys)])}
          items={[
            {
              key: "raw",
              label: "Raw Feeds (Debug)",
              children: (
                <Row gutter={[16, 16]}>
                  <Col xs={24} xl={12}>
                    <Card className="hmi-card" size="small" title="OAK RGB Raw">
                      <div className="camera-frame">
                        <img
                          src={`/stream/camera.mjpeg?topic=${encodeURIComponent(
                            cameraDraft || robot.selectedCameraTopic || "",
                          )}`}
                          alt="oak-rgb-raw"
                        />
                      </div>
                      <div style={{ marginTop: 8 }}>
                        <Text type="secondary">Topic: {status?.visualizer?.rgb?.topic || "n/a"}</Text>
                        <br />
                        <Text type="secondary">
                          hz {hz(status?.visualizer?.rgb?.fps)} | age {secAge(status?.visualizer?.rgb?.age_sec)}
                        </Text>
                        <br />
                        <Text type="secondary">
                          encoding {status?.visualizer?.rgb?.encoding || "n/a"} | frame {status?.visualizer?.rgb?.frame_id || "n/a"}
                        </Text>
                      </div>
                    </Card>
                  </Col>
                  <Col xs={24} xl={12}>
                    <Card className="hmi-card" size="small" title="LiDAR Raw">
                      <LidarCanvas scan={scanPayload} />
                      <div style={{ marginTop: 8 }}>
                        <Text type="secondary">Topic: {status?.visualizer?.scan?.topic || "n/a"}</Text>
                        <br />
                        <Text type="secondary">
                          hz {hz(status?.visualizer?.scan?.fps)} | age {secAge(status?.visualizer?.scan?.age_sec)}
                        </Text>
                        <br />
                        <Text type="secondary">
                          angle [{(status?.visualizer?.scan?.angle_min || 0).toFixed(2)}, {(status?.visualizer?.scan?.angle_max || 0).toFixed(2)}] | range [{(status?.visualizer?.scan?.range_min || 0).toFixed(2)}, {(status?.visualizer?.scan?.range_max || 0).toFixed(2)}]
                        </Text>
                        <br />
                        <Text type="secondary">
                          samples {status?.visualizer?.scan?.sample_count || 0} | frame {status?.visualizer?.scan?.frame_id || "n/a"}
                        </Text>
                      </div>
                    </Card>
                  </Col>
                </Row>
              ),
            },
          ]}
        />
      </Space>
    );
  };

  const renderManual = (): JSX.Element => (
    <Row gutter={[16, 16]}>
      <Col xs={24} lg={12}>
        <Card title="Deadman + Directional Control" className="hmi-card">
          <Space direction="vertical" style={{ width: "100%" }} size={12}>
            <Alert
              type={motionActive ? "success" : "warning"}
              showIcon
              message={motionActive ? "Motion enabled" : "Motion disabled"}
              description={motionActive ? `Window remaining: ${deadmanCountdown.toFixed(1)}s` : "Press Enable Motion (5s)"}
            />
            <Button type="primary" onClick={() => runAction("Motion enabled", armDeadman)}>
              Enable Motion (5s)
            </Button>
            <div>
              <Text>Speed limit</Text>
              <Slider min={0.1} max={1.0} step={0.05} value={driveSpeed} onChange={(v) => setDriveSpeed(Number(v))} />
            </div>
            <div className="drive-grid">
              <Button
                onMouseDown={() => startDrive("forward")}
                onMouseUp={stopDrive}
                onMouseLeave={stopDrive}
                onTouchStart={() => startDrive("forward")}
                onTouchEnd={stopDrive}
              >
                Forward
              </Button>
              <Button
                onMouseDown={() => startDrive("left")}
                onMouseUp={stopDrive}
                onMouseLeave={stopDrive}
                onTouchStart={() => startDrive("left")}
                onTouchEnd={stopDrive}
              >
                Left
              </Button>
              <Button danger onMouseDown={stopDrive} onTouchStart={stopDrive}>
                Stop
              </Button>
              <Button
                onMouseDown={() => startDrive("right")}
                onMouseUp={stopDrive}
                onMouseLeave={stopDrive}
                onTouchStart={() => startDrive("right")}
                onTouchEnd={stopDrive}
              >
                Right
              </Button>
              <Button
                onMouseDown={() => startDrive("backward")}
                onMouseUp={stopDrive}
                onMouseLeave={stopDrive}
                onTouchStart={() => startDrive("backward")}
                onTouchEnd={stopDrive}
              >
                Backward
              </Button>
            </div>
            <Text type="secondary">Keyboard shortcuts: W/A/S/D (hold key to move)</Text>
          </Space>
        </Card>
      </Col>
      <Col xs={24} lg={12}>
        <Card title="Motor Bank Control" className="hmi-card">
          <Space direction="vertical" style={{ width: "100%" }} size={12}>
            <div>
              <Text>Left bank: {leftBank}</Text>
              <Slider min={-100} max={100} step={1} value={leftBank} onChange={(value) => setLeftBank(Number(value))} />
            </div>
            <div>
              <Text>Right bank: {rightBank}</Text>
              <Slider min={-100} max={100} step={1} value={rightBank} onChange={(value) => setRightBank(Number(value))} />
            </div>
            <Space>
              <Button type="primary" onClick={() => runAction("Motor bank command sent", sendMotorBank)}>
                Send Bank Command
              </Button>
              <Button
                onClick={() => {
                  setLeftBank(0);
                  setRightBank(0);
                  runAction("Bank centered", () => sendBase({ type: "stop" }));
                }}
              >
                Center + Stop
              </Button>
            </Space>
            <Alert type="info" showIcon message="Deadman required" description="Motor bank commands require Manual mode and armed motion window." />
          </Space>
        </Card>
      </Col>
    </Row>
  );

  const renderArm = (): JSX.Element => (
    <Space direction="vertical" size={16} style={{ width: "100%" }}>
      <Card title="Per-Joint Control" className="hmi-card">
        {!armJoints.length ? (
          <Empty description="No arm joints configured" />
        ) : (
          <Space direction="vertical" style={{ width: "100%" }} size={14}>
            {armJoints.map((joint) => (
              <div key={joint.name}>
                <Text>{joint.name}: {(jointValues[joint.name] ?? 0).toFixed(3)}</Text>
                <Slider
                  min={joint.min}
                  max={joint.max}
                  step={joint.step}
                  value={jointValues[joint.name] ?? joint.default ?? 0}
                  onChange={(value) =>
                    setJointValues((prev) => ({
                      ...prev,
                      [joint.name]: Number(value),
                    }))
                  }
                />
              </div>
            ))}
          </Space>
        )}
        <Divider />
        <Space wrap>
          <Button type="primary" onClick={() => runAction("Joint command published", sendArmJoints)}>
            Send Joint Setpoints
          </Button>
          <Button onClick={() => runAction("Arm stopped", () => postApi("/api/manual/arm", { action: "stop" }))} danger>
            Stop Arm Now
          </Button>
        </Space>
      </Card>

      <Card title="Gripper + Named Poses" className="hmi-card">
        <Space wrap>
          <Button onClick={() => runAction("Gripper open command sent", () => postApi("/api/manual/arm", { gripper: "open" }))}>Gripper Open</Button>
          <Button onClick={() => runAction("Gripper close command sent", () => postApi("/api/manual/arm", { gripper: "close" }))}>Gripper Close</Button>
          {namedPoses.map((pose) => (
            <Button key={pose} onClick={() => runAction(`Pose ${pose} sent`, () => postApi("/api/manual/arm", { pose }))}>
              {pose}
            </Button>
          ))}
        </Space>
      </Card>
    </Space>
  );

  const renderTraining = (): JSX.Element => {
    const previewTopic = cameraDraft || robot.selectedCameraTopic || "";

    return (
      <Space direction="vertical" size={16} style={{ width: "100%" }}>
        <Card title="Pre-Capture Camera Preview" className="hmi-card">
          {previewTopic ? (
            <div className="camera-frame">
              <img
                src={`/stream/camera.mjpeg?topic=${encodeURIComponent(previewTopic)}`}
                alt="training-preview"
              />
            </div>
          ) : (
            <Empty description="No camera stream topic selected" />
          )}
          <Space wrap style={{ marginTop: 10 }}>
            <Select
              style={{ minWidth: 280 }}
              value={cameraDraft || undefined}
              placeholder="Camera MJPEG topic"
              options={cameraStreamTopics.map((topic) => ({ value: topic, label: topic }))}
              onChange={(value) => setCameraDraft(value)}
            />
            <Button onClick={() => runAction("Camera topic selected", applyCameraTopic)}>
              Apply Camera Stream
            </Button>
          </Space>
          <div style={{ marginTop: 8 }}>
            <Text type="secondary">Topic: {previewTopic || "n/a"}</Text>
            <br />
            <Text type="secondary">
              hz {hz(status?.visualizer?.camera?.fps)} | age {secAge(status?.visualizer?.camera?.age_sec)}
            </Text>
          </div>
        </Card>

        <Card title="Training + Data Capture" className="hmi-card">
          <Space direction="vertical" size={12} style={{ width: "100%" }}>
            <Text>Run guided capture by target image count, or set an explicit duration override.</Text>
            <Space.Compact style={{ width: "100%" }}>
              <Input
                value={trainingTag}
                onChange={(event) => setTrainingTag(event.target.value)}
                placeholder="dataset tag (e.g. s07_mixed_clutter)"
              />
              <Button
                type="primary"
                onClick={() =>
                  runAction("Timed capture started", () =>
                    postApi("/api/recording/capture_plan_start", {
                      tags: trainingTag,
                      target_images: captureTargetImages,
                      extraction_fps: captureExtractionFps,
                      safety_buffer_sec: captureBufferSec,
                      duration_sec: captureDurationOverrideSec,
                    }),
                  )
                }
              >
                Start Timed Capture
              </Button>
              <Button
                onClick={() => runAction("Manual capture started", () => postApi("/api/recording/start", { tags: trainingTag }))}
              >
                Start Manual
              </Button>
              <Button onClick={() => runAction("Capture stopped", () => postApi("/api/recording/stop", {}))}>Stop</Button>
            </Space.Compact>
            <Space wrap size={16}>
              <Space>
                <Text>Target images</Text>
                <InputNumber min={1} max={5000} value={captureTargetImages} onChange={(v) => setCaptureTargetImages(Number(v || 1))} />
              </Space>
              <Space>
                <Text>Extractor FPS</Text>
                <InputNumber
                  min={0.1}
                  max={30}
                  step={0.1}
                  value={captureExtractionFps}
                  onChange={(v) => setCaptureExtractionFps(Number(v || 2))}
                />
              </Space>
              <Space>
                <Text>Buffer (sec)</Text>
                <InputNumber min={0} max={120} value={captureBufferSec} onChange={(v) => setCaptureBufferSec(Number(v || 0))} />
              </Space>
              <Space>
                <Text>Duration override (sec)</Text>
                <InputNumber
                  min={0}
                  max={3600}
                  value={captureDurationOverrideSec}
                  onChange={(v) => setCaptureDurationOverrideSec(Number(v || 0))}
                />
              </Space>
            </Space>
            <Alert
              type="info"
              showIcon
              message={`Planned capture runtime: ${plannedCaptureDurationSec.toFixed(1)} sec`}
              description={
                captureDurationOverrideSec > 0
                  ? "Duration override is active. Target image estimate is not used for runtime."
                  : "Runtime is estimated as target_images / extractor_fps + buffer."
              }
            />
            <Space direction="vertical" size={4} style={{ width: "100%" }}>
              <Text type="secondary">Bag storage root: {bagStoragePath}</Text>
              <Text type="secondary">Active bag output: {activeOutputDir || "n/a"}</Text>
              <Text type="secondary">Extracted image output: {extractedOutputPath}</Text>
              <Text type="secondary">
                Capture plan state: {capturePlan?.state || "idle"}
                {capturePlan?.active ? ` (remaining ${captureRemainingSec.toFixed(1)} sec)` : ""}
              </Text>
            </Space>
            <Button
              onClick={() =>
                runAction("Replay hint fetched", async () => {
                  const rec = status?.recording?.recent?.[0];
                  const out = await postApi<{ command?: string }>("/api/recording/replay_hint", {
                    path: rec?.path || "",
                  });
                  const command = String(out.command || "");
                  if (command) {
                    await navigator.clipboard.writeText(command);
                    messageApi.success("Replay command copied to clipboard");
                  }
                })
              }
            >
              Copy Replay Command (latest)
            </Button>
          </Space>
        </Card>
      </Space>
    );
  };

  const renderDemo = (): JSX.Element => (
    <Card title="Demo Execution" className="hmi-card">
      <Space direction="vertical" size={12} style={{ width: "100%" }}>
        <Space>
          <Text>Run count</Text>
          <InputNumber min={1} max={100} value={demoCount} onChange={(value) => setDemoCount(Number(value || 1))} />
          <Button type="primary" onClick={() => runAction("Demo run started", () => postApi("/api/demo/run", { count: demoCount }))}>
            Run Demo
          </Button>
          <Button danger onClick={() => runAction("Demo run stopped", () => postApi("/api/demo/stop", {}))}>
            Stop
          </Button>
        </Space>
        <Progress
          percent={
            status?.demo?.target_count
              ? Math.round((Number(status.demo.completed_count || 0) / Number(status.demo.target_count || 1)) * 100)
              : 0
          }
          status={status?.demo?.active ? "active" : "normal"}
        />
        <Text type="secondary">
          Completed {status?.demo?.completed_count || 0} / {status?.demo?.target_count || 0}
        </Text>
      </Space>
    </Card>
  );

  const renderLogs = (): JSX.Element => (
    <Space direction="vertical" size={12} style={{ width: "100%" }}>
      <Card className="hmi-card" title="Live Service Logs">
        <Space wrap style={{ marginBottom: 12 }}>
          <Select
            value={logSource}
            onChange={setLogSource}
            options={(logSources.length ? logSources : [{ id: "robot_console", path: "", exists: true, dynamic: false }]).map((source) => ({
              value: source.id,
              label: `${source.id}${source.dynamic ? " (dynamic)" : ""}`,
            }))}
            style={{ minWidth: 220 }}
          />
          <Segmented
            value={logSeverity}
            onChange={(value) => setLogSeverity(value as SeverityFilter)}
            options={[
              { value: "all", label: "All" },
              { value: "info", label: "Info" },
              { value: "warn", label: "Warn" },
              { value: "error", label: "Error" },
            ]}
          />
          <Input
            placeholder="Filter text"
            value={logQuery}
            onChange={(event) => setLogQuery(event.target.value)}
            style={{ width: 220 }}
          />
          <Button onClick={() => setLogEntries([])}>Clear</Button>
          <Button
            onClick={() =>
              runAction("Diagnostics copied", async () => {
                const text = await fetch("/api/logs/diagnostics?lines=80").then((r) => r.text());
                await navigator.clipboard.writeText(text);
              })
            }
          >
            Copy Diagnostics
          </Button>
          <Button href="/api/logs/diagnostics?download=1&lines=120" target="_blank">
            Download Diagnostics
          </Button>
        </Space>

        <div className="log-view">
          <List
            dataSource={filteredLogs}
            renderItem={(entry) => (
              <List.Item className="log-row">
                <Space size={8}>
                  <Text type="secondary" style={{ fontFamily: "monospace" }}>
                    {new Date(entry.ts * 1000).toLocaleTimeString()}
                  </Text>
                  <Tag>{entry.source}</Tag>
                  {severityTag(entry.severity)}
                </Space>
                <Text style={{ fontFamily: "monospace" }}>{entry.line || "(empty)"}</Text>
              </List.Item>
            )}
          />
        </div>
      </Card>
    </Space>
  );

  const renderPanel = (): JSX.Element => {
    switch (ui.navKey as MenuKey) {
      case "visualizer":
        return renderVisualizer();
      case "manual":
        return renderManual();
      case "unified":
        return <UnifiedControlPage status={status} wsConnected={transport.wsConnected} />;
      case "arm":
        return renderArm();
      case "training":
        return renderTraining();
      case "demo":
        return renderDemo();
      case "logs":
        return renderLogs();
      case "operations":
      default:
        return renderOperations();
    }
  };

  const unifiedArmSafe = transport.wsConnected && Number(status?.arm?.joints?.length || 0) > 0;

  return (
    <>
      {contextHolder}
      <Layout style={{ minHeight: "100vh" }}>
        <Sider theme="dark" width={256} breakpoint="lg" collapsedWidth={72}>
          <div className="hmi-brand">
            <Title level={4} style={{ color: "#fff", margin: 0 }}>Robot Console</Title>
            <Text style={{ color: "#8aa6c1" }}>Factory HMI</Text>
          </div>
          <Menu
            theme="dark"
            mode="inline"
            selectedKeys={[ui.navKey]}
            onClick={({ key }) => setNavKey(key)}
            items={baseMenuItems}
          />
        </Sider>

        <Layout>
          <Header className={`hmi-header ${ui.navKey === "unified" ? "unified-header" : ""}`}>
            {ui.navKey === "unified" ? (
              <div className="unified-header-inner">
                <div className="unified-header-title">
                  <span className="title-main">Robot Console</span>
                  <span className="title-dot">{"\u2022"}</span>
                  <span className="title-sub">Unified Control</span>
                </div>

                <div className="unified-header-controls">
                  <Select
                    style={{ minWidth: 190 }}
                    value={robot.selectedMode}
                    options={MODE_OPTIONS}
                    onChange={(value) => {
                      applyModeImmediate(value).catch(() => undefined);
                    }}
                  />
                  <Tag className="unified-top-badge">Base: {status?.safety?.armed ? "ARMED" : "DISARMED"}</Tag>
                  <Tag className="unified-top-badge">Arm: {unifiedArmSafe ? "SAFE" : "OFFLINE"}</Tag>
                  <Button className="unified-top-disarm" onClick={() => disarmUnified().catch(() => undefined)}>
                    DISARM
                  </Button>
                </div>
              </div>
            ) : (
              <>
                <div className="hmi-header-left">
                  <Space align="center">
                    <Badge status={transport.wsConnected ? "success" : "error"} text={transport.wsConnected ? "Backend Connected" : transport.wsStateLabel} />
                    <Text type="secondary">Last update: {transport.lastUpdateUnixMs ? new Date(transport.lastUpdateUnixMs).toLocaleTimeString() : "never"}</Text>
                  </Space>
                </div>

                <div className="hmi-header-center">
                  <Space>
                    <Select
                      style={{ minWidth: 190 }}
                      value={modeDraft}
                      options={MODE_OPTIONS}
                      onChange={(value) => {
                        setModeDraft(value);
                        setModeDirty(true);
                      }}
                    />
                    <Button
                      type="primary"
                      icon={<ControlOutlined />}
                      disabled={modeDraft === robot.selectedMode}
                      onClick={() => runAction("Mode updated", applyMode)}
                    >
                      Apply Mode
                    </Button>
                  </Space>
                </div>

                <div className="hmi-header-right">
                  <Space>
                    <Tag color={status?.safety?.armed ? "success" : "default"}>{status?.safety?.armed ? "ARMED" : "DISARMED"}</Tag>
                    <Tag color={status?.safety?.estop_latched ? "error" : "success"}>{status?.safety?.estop_latched ? "E-STOP" : "CLEAR"}</Tag>
                    <Button onClick={toggleRightPanel}>{ui.rightPanelOpen ? "Hide Telemetry" : "Show Telemetry"}</Button>
                  </Space>
                </div>
              </>
            )}
          </Header>

          <Content className={`hmi-content ${ui.navKey === "unified" ? "hmi-content-unified" : ""}`}>{renderPanel()}</Content>
        </Layout>

        {ui.rightPanelOpen && ui.navKey !== "unified" ? (
          <Sider width={340} theme="dark" className="hmi-right-panel">
            <div className="hmi-right-inner">
              <Title level={5} style={{ color: "#fff", marginTop: 0 }}>Live Telemetry</Title>
              <Space direction="vertical" style={{ width: "100%" }} size={10}>
                <Card size="small" className="hmi-mini-card" title="Connection">
                  <Space direction="vertical" size={2}>
                    <Text>WebSocket: <Tag color={transport.wsConnected ? "success" : "error"}>{transport.wsConnected ? "online" : "offline"}</Tag></Text>
                    <Text>WS clients: {status?.connection?.ws_clients || 0}</Text>
                  </Space>
                </Card>

                <Card size="small" className="hmi-mini-card" title="Sensors">
                  <Row gutter={[8, 8]}>
                    <Col span={12}><Statistic title="Lidar" value={status?.visualizer?.scan?.fps || 0} precision={1} suffix="Hz" /></Col>
                    <Col span={12}><Statistic title="Camera" value={status?.visualizer?.camera?.fps || 0} precision={1} suffix="Hz" /></Col>
                    <Col span={12}><Statistic title="Map" value={status?.visualizer?.map?.fps || 0} precision={1} suffix="Hz" /></Col>
                    <Col span={12}><Statistic title="PointCloud" value={status?.visualizer?.pointcloud?.fps || 0} precision={1} suffix="Hz" /></Col>
                    <Col span={12}><Text type="secondary">scan age {secAge(status?.visualizer?.scan?.age_sec)}</Text></Col>
                    <Col span={12}><Text type="secondary">cam age {secAge(status?.visualizer?.camera?.age_sec)}</Text></Col>
                    <Col span={12}><Text type="secondary">map age {secAge(status?.visualizer?.map?.age_sec)}</Text></Col>
                    <Col span={12}><Text type="secondary">cloud age {secAge(status?.visualizer?.pointcloud?.age_sec)}</Text></Col>
                  </Row>
                </Card>

                <Card size="small" className="hmi-mini-card" title="Safety">
                  <Space direction="vertical" size={2}>
                    <Text>Watchdog: <Tag color={status?.safety?.watchdog_ok ? "success" : "warning"}>{status?.safety?.watchdog_ok ? "OK" : "TRIPPED"}</Tag></Text>
                    <Text>Timeout: {status?.safety?.command_timeout_sec || 0}s</Text>
                    <Text>Last command age: {secAge(status?.safety?.last_command_age_sec)}</Text>
                    <Text>Last disarm reason: {status?.safety?.last_disarm_reason || "n/a"}</Text>
                  </Space>
                </Card>

                <Card size="small" className="hmi-mini-card" title="Thermals + Storage">
                  <Space direction="vertical" size={2}>
                    <Text>CPU temp: {(status?.reliability?.cpu_temp_c || 0).toFixed(1)} C</Text>
                    <Text>Max temp: {(status?.reliability?.max_temp_c || 0).toFixed(1)} C</Text>
                    <Text>Disk free: {bytes(status?.reliability?.disk_free_bytes || 0)}</Text>
                    <Text>Disk used: {bytes(status?.reliability?.disk_used_bytes || 0)}</Text>
                  </Space>
                </Card>

                <Card size="small" className="hmi-mini-card" title="Navigation">
                  <Space direction="vertical" size={2}>
                    <Text>State: {status?.navigation?.state || "idle"}</Text>
                    <Text>Last result: {status?.navigation?.last_result || "none"}</Text>
                    <Text>Base odom age: {secAge(status?.health?.odom_age_sec)}</Text>
                    <Text>Lidar age: {secAge(status?.health?.scan_age_sec)}</Text>
                    <Text>Encoder update: {agoUnix(status?.dashboard?.last_encoder_update_unix)}</Text>
                  </Space>
                </Card>

                <Card size="small" className="hmi-mini-card" title="Foxglove">
                  <Text code>{`ws://${window.location.hostname}:8765`}</Text>
                  <Divider style={{ margin: "8px 0" }} />
                  <Button size="small" href={`https://studio.foxglove.dev/?ds=foxglove-websocket&ds.url=${encodeURIComponent(`ws://${window.location.hostname}:8765`)}`} target="_blank">
                    Connect
                  </Button>
                </Card>
              </Space>
            </div>
          </Sider>
        ) : null}
      </Layout>
    </>
  );
}
