import {
  Alert,
  Badge,
  Button,
  Card,
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
import { MapCanvas } from "./components/MapCanvas";
import { PointCloudCanvas } from "./components/PointCloudCanvas";
import { getStatus, postApi } from "./lib/api";
import { agoUnix, bytes, hz, secAge, timestampToLocal } from "./lib/format";
import { getCachedCameraTopic, getCachedMode, useConsoleStore } from "./store/useConsoleStore";
import type {
  ArmJointConfig,
  LogSource,
  MapOverlayPayload,
  MapPayload,
  PointcloudPayload,
  RobotMode,
  ScanPayload,
  StatusPayload,
  WsEvent,
} from "./types";

const { Header, Content, Sider } = Layout;
const { Title, Text } = Typography;

type MenuKey = "operations" | "visualizer" | "manual" | "arm" | "training" | "demo" | "logs";
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
    setMapOverlay,
    setPointcloudPayload,
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
  const [pointcloudDraft, setPointcloudDraft] = useState("");
  const [driveSpeed, setDriveSpeed] = useState(0.32);
  const [motionDeadlineMs, setMotionDeadlineMs] = useState(0);
  const [deadmanCountdown, setDeadmanCountdown] = useState(0);
  const [leftBank, setLeftBank] = useState(0);
  const [rightBank, setRightBank] = useState(0);
  const [jointValues, setJointValues] = useState<Record<string, number>>({});
  const [trainingTag, setTrainingTag] = useState("training");
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
  const logSources: LogSource[] = status?.logs?.sources || [];
  const cameraTopics = status?.visualizer?.camera?.available_topics || [];
  const mapTopic = status?.visualizer?.map?.topic || "";
  const pathTopic = status?.visualizer?.map?.selected_path_topic || "";
  const pointcloudTopics = status?.visualizer?.pointcloud?.available_topics || [];
  const armJoints = (status?.arm?.joints || []) as ArmJointConfig[];
  const namedPoses = status?.arm?.named_poses || [];

  const motionActive = motionDeadlineMs > Date.now();
  const mapPayload = robot.mapPayload;
  const mapOverlay = robot.mapOverlay;
  const pointcloudPayload = robot.pointcloudPayload;

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
        if (payload.type === "map_overlay") {
          setMapOverlay(payload.data as MapOverlayPayload);
          return;
        }
        if (payload.type === "pointcloud") {
          setPointcloudPayload(payload.data as PointcloudPayload);
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
    if (!pointcloudDraft && status?.visualizer?.pointcloud?.selected_topic) {
      setPointcloudDraft(status.visualizer.pointcloud.selected_topic);
    }
  }, [pointcloudDraft, status?.visualizer?.pointcloud?.selected_topic]);

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

  const applyCameraTopic = useCallback(async () => {
    if (!cameraDraft) {
      return;
    }
    await postApi("/api/camera/select", { topic: cameraDraft });
    setSelectedCameraTopic(cameraDraft);
    await refreshStatus();
  }, [cameraDraft, refreshStatus, setSelectedCameraTopic]);

  const applyMapConfig = useCallback(async () => {
    await postApi("/api/map/config", {
      map_topic: mapTopicDraft || undefined,
      path_topic: pathTopicDraft || undefined,
    });
    await refreshStatus();
  }, [mapTopicDraft, pathTopicDraft, refreshStatus]);

  const applyPointcloudTopic = useCallback(async () => {
    if (!pointcloudDraft) {
      return;
    }
    await postApi("/api/pointcloud/select", { topic: pointcloudDraft });
    await refreshStatus();
  }, [pointcloudDraft, refreshStatus]);

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

  const baseMenuItems = [
    { key: "operations", icon: <DashboardOutlined />, label: "Operations" },
    { key: "visualizer", icon: <RadarChartOutlined />, label: "Visualizer" },
    { key: "manual", icon: <ControlOutlined />, label: "Manual Base" },
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
    const mapTopicOptions = status?.visualizer?.map?.available_map_topics || [];
    const pathTopicOptions = status?.visualizer?.map?.available_path_topics || [];
    const pose = mapOverlay?.robot_pose || status?.visualizer?.map?.pose;

    return (
      <Row gutter={[16, 16]}>
        <Col xs={24} xxl={14}>
          <Card
            className="hmi-card"
            title="Waymo-Style 2D Mapping View"
            extra={
              <Space>
                <Badge
                  status={status?.visualizer?.map?.connected ? "success" : "error"}
                  text={status?.visualizer?.map?.connected ? "/map Live" : "/map Waiting"}
                />
                <Text type="secondary">scan {hz(status?.visualizer?.scan?.fps)}</Text>
                <Text type="secondary">tf age {secAge(status?.visualizer?.map?.tf_age_sec)}</Text>
              </Space>
            }
          >
            <Space.Compact style={{ width: "100%", marginBottom: 12 }}>
              <Select
                style={{ width: "45%" }}
                value={mapTopicDraft || undefined}
                placeholder="Map topic"
                options={mapTopicOptions.map((topic) => ({ value: topic, label: topic }))}
                onChange={(value) => setMapTopicDraft(value)}
              />
              <Select
                style={{ width: "45%" }}
                value={pathTopicDraft || undefined}
                placeholder="Path topic"
                options={pathTopicOptions.map((topic) => ({ value: topic, label: topic }))}
                onChange={(value) => setPathTopicDraft(value)}
              />
              <Button onClick={() => runAction("Map config applied", applyMapConfig)}>Apply</Button>
            </Space.Compact>
            <MapCanvas map={mapPayload} overlay={mapOverlay} />
            <Row gutter={[12, 8]} style={{ marginTop: 10 }}>
              <Col xs={24} md={12}>
                <Text type="secondary">
                  Pose:{" "}
                  {pose
                    ? `${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}, yaw ${pose.yaw.toFixed(2)}`
                    : "n/a"}
                </Text>
              </Col>
              <Col xs={24} md={12}>
                <Text type="secondary">
                  Path: {pathTopicDraft || pathTopic || "n/a"} ({mapOverlay?.path_point_count || 0} pts)
                </Text>
              </Col>
              <Col xs={24} md={12}>
                <Text type="secondary">
                  Map: {mapPayload?.width || 0}x{mapPayload?.height || 0} @{" "}
                  {(mapPayload?.resolution || 0).toFixed(3)} m/px
                </Text>
              </Col>
              <Col xs={24} md={12}>
                <Text type="secondary">
                  Scan overlay: {mapOverlay?.scan_point_count || 0} pts, age{" "}
                  {secAge(mapOverlay?.scan_age_sec)}
                </Text>
              </Col>
            </Row>
          </Card>
        </Col>

        <Col xs={24} xxl={10}>
          <Space direction="vertical" size={16} style={{ width: "100%" }}>
            <Card
              className="hmi-card"
              title="Live 3D Point Cloud"
              extra={
                <Space>
                  <Badge
                    status={status?.visualizer?.pointcloud?.connected ? "success" : "error"}
                    text={status?.visualizer?.pointcloud?.connected ? "Live" : "Waiting"}
                  />
                  <Text type="secondary">{hz(status?.visualizer?.pointcloud?.fps)}</Text>
                </Space>
              }
            >
              <Space.Compact style={{ width: "100%", marginBottom: 12 }}>
                <Select
                  style={{ width: "100%" }}
                  value={pointcloudDraft || undefined}
                  placeholder="PointCloud2 topic"
                  options={pointcloudTopics.map((topic) => ({ value: topic, label: topic }))}
                  onChange={(value) => setPointcloudDraft(value)}
                />
                <Button onClick={() => runAction("Pointcloud topic selected", applyPointcloudTopic)}>
                  Apply
                </Button>
              </Space.Compact>
              <PointCloudCanvas cloud={pointcloudPayload} />
              <div style={{ marginTop: 8 }}>
                <Text type="secondary">
                  Topic: {pointcloudDraft || status?.visualizer?.pointcloud?.selected_topic || "n/a"}
                </Text>
                <br />
                <Text type="secondary">
                  Age: {secAge(status?.visualizer?.pointcloud?.age_sec)} | Points:{" "}
                  {pointcloudPayload?.point_count || 0}
                </Text>
              </div>
            </Card>

            <Card
              className="hmi-card"
              title="OAK-D RGB"
              extra={
                <Space>
                  <Badge
                    status={status?.visualizer?.camera?.connected ? "success" : "error"}
                    text={status?.visualizer?.camera?.connected ? "Live" : "Disconnected"}
                  />
                  <Text type="secondary">{hz(status?.visualizer?.camera?.fps)}</Text>
                </Space>
              }
            >
              <Space.Compact style={{ width: "100%", marginBottom: 12 }}>
                <Select
                  style={{ width: "100%" }}
                  value={cameraDraft || undefined}
                  placeholder="Select camera topic"
                  options={cameraTopics.map((topic) => ({ value: topic, label: topic }))}
                  onChange={(value) => setCameraDraft(value)}
                />
                <Button onClick={() => runAction("Camera topic selected", applyCameraTopic)}>Apply</Button>
              </Space.Compact>
              <div className="camera-frame">
                <img
                  src={`/stream/camera.mjpeg?topic=${encodeURIComponent(
                    cameraDraft || robot.selectedCameraTopic || "",
                  )}`}
                  alt="camera"
                />
              </div>
              <div style={{ marginTop: 8 }}>
                <Text type="secondary">Topic: {cameraDraft || robot.selectedCameraTopic || "n/a"}</Text>
                <br />
                <Text type="secondary">Age: {secAge(status?.visualizer?.camera?.age_sec)}</Text>
              </div>
              <Divider />
              <Button
                icon={<CameraOutlined />}
                href={`https://studio.foxglove.dev/?ds=foxglove-websocket&ds.url=${encodeURIComponent(
                  `ws://${window.location.hostname}:8765`,
                )}`}
                target="_blank"
              >
                Open Foxglove (ws://{window.location.hostname}:8765)
              </Button>
            </Card>
          </Space>
        </Col>
      </Row>
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

  const renderTraining = (): JSX.Element => (
    <Card title="Training + Data Capture" className="hmi-card">
      <Space direction="vertical" size={12} style={{ width: "100%" }}>
        <Text>Capture datasets with tags, then use replay hint for offline model training loops.</Text>
        <Space.Compact style={{ width: "100%" }}>
          <Input value={trainingTag} onChange={(event) => setTrainingTag(event.target.value)} placeholder="dataset tag" />
          <Button type="primary" onClick={() => runAction("Training bag started", () => postApi("/api/recording/start", { tags: trainingTag }))}>Start Capture</Button>
          <Button onClick={() => runAction("Training bag stopped", () => postApi("/api/recording/stop", {}))}>Stop Capture</Button>
        </Space.Compact>
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
  );

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
          <Header className="hmi-header">
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
          </Header>

          <Content className="hmi-content">{renderPanel()}</Content>
        </Layout>

        {ui.rightPanelOpen ? (
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
