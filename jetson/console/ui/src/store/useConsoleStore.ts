import { create } from "zustand";
import type {
  DepthPayload,
  DetectionPayload,
  MapOverlayPayload,
  MapPayload,
  PointcloudPayload,
  RobotMode,
  ScanPayload,
  StatusPayload,
} from "../types";

const MODE_KEY = "robot_console_hmi.mode";
const CAMERA_TOPIC_KEY = "robot_console_hmi.camera_topic";
const NAV_KEY = "robot_console_hmi.nav";

function readStorage(key: string, fallback = ""): string {
  try {
    return window.localStorage.getItem(key) || fallback;
  } catch {
    return fallback;
  }
}

function writeStorage(key: string, value: string): void {
  try {
    window.localStorage.setItem(key, value);
  } catch {
    // no-op
  }
}

type TransportState = {
  wsConnected: boolean;
  wsStateLabel: string;
  lastUpdateUnixMs: number;
  reconnectCounter: number;
};

type RobotState = {
  status: StatusPayload | null;
  scanPayload: ScanPayload | null;
  mapPayload: MapPayload | null;
  mapOverlay: MapOverlayPayload | null;
  pointcloudPayload: PointcloudPayload | null;
  depthPayload: DepthPayload | null;
  detectionPayload: DetectionPayload | null;
  selectedMode: RobotMode;
  selectedCameraTopic: string;
  modeReconciled: boolean;
};

type UiState = {
  navKey: string;
  rightPanelOpen: boolean;
  compactTelemetry: boolean;
};

type ConsoleStore = {
  transport: TransportState;
  robot: RobotState;
  ui: UiState;
  setWsConnected: (connected: boolean) => void;
  setWsStateLabel: (label: string) => void;
  setStatus: (status: StatusPayload) => void;
  setScanPayload: (scan: ScanPayload) => void;
  setMapPayload: (map: MapPayload) => void;
  setMapOverlay: (overlay: MapOverlayPayload) => void;
  setPointcloudPayload: (payload: PointcloudPayload) => void;
  setDepthPayload: (payload: DepthPayload) => void;
  setDetectionPayload: (payload: DetectionPayload) => void;
  setSelectedMode: (mode: RobotMode, persist?: boolean) => void;
  setSelectedCameraTopic: (topic: string, persist?: boolean) => void;
  setModeReconciled: (value: boolean) => void;
  setNavKey: (key: string) => void;
  toggleRightPanel: () => void;
};

export const useConsoleStore = create<ConsoleStore>((set) => ({
  transport: {
    wsConnected: false,
    wsStateLabel: "disconnected",
    lastUpdateUnixMs: 0,
    reconnectCounter: 0
  },
  robot: {
    status: null,
    scanPayload: null,
    mapPayload: null,
    mapOverlay: null,
    pointcloudPayload: null,
    depthPayload: null,
    detectionPayload: null,
    selectedMode: (readStorage(MODE_KEY, "manual") as RobotMode) || "manual",
    selectedCameraTopic: readStorage(CAMERA_TOPIC_KEY, ""),
    modeReconciled: false
  },
  ui: {
    navKey: readStorage(NAV_KEY, "operations"),
    rightPanelOpen: true,
    compactTelemetry: false
  },
  setWsConnected: (connected) => {
    set((state) => ({
      transport: {
        ...state.transport,
        wsConnected: connected,
        wsStateLabel: connected ? "connected" : "disconnected",
        reconnectCounter: connected ? state.transport.reconnectCounter : state.transport.reconnectCounter + 1
      }
    }));
  },
  setWsStateLabel: (label) => {
    set((state) => ({ transport: { ...state.transport, wsStateLabel: label } }));
  },
  setStatus: (status) => {
    const backendMode = status.mode;
    const selectedTopic = status.visualizer?.camera?.selected_topic || "";
    set((state) => ({
      robot: {
        ...state.robot,
        status,
        selectedMode: backendMode,
        selectedCameraTopic: selectedTopic || state.robot.selectedCameraTopic
      },
      transport: {
        ...state.transport,
        lastUpdateUnixMs: Date.now()
      }
    }));
    writeStorage(MODE_KEY, backendMode);
    if (selectedTopic) {
      writeStorage(CAMERA_TOPIC_KEY, selectedTopic);
    }
  },
  setScanPayload: (scan) => {
    set((state) => ({ robot: { ...state.robot, scanPayload: scan } }));
  },
  setMapPayload: (map) => {
    set((state) => ({ robot: { ...state.robot, mapPayload: map } }));
  },
  setMapOverlay: (overlay) => {
    set((state) => ({ robot: { ...state.robot, mapOverlay: overlay } }));
  },
  setPointcloudPayload: (payload) => {
    set((state) => ({ robot: { ...state.robot, pointcloudPayload: payload } }));
  },
  setDepthPayload: (payload) => {
    set((state) => ({ robot: { ...state.robot, depthPayload: payload } }));
  },
  setDetectionPayload: (payload) => {
    set((state) => ({ robot: { ...state.robot, detectionPayload: payload } }));
  },
  setSelectedMode: (mode, persist = true) => {
    set((state) => ({ robot: { ...state.robot, selectedMode: mode } }));
    if (persist) {
      writeStorage(MODE_KEY, mode);
    }
  },
  setSelectedCameraTopic: (topic, persist = true) => {
    set((state) => ({ robot: { ...state.robot, selectedCameraTopic: topic } }));
    if (persist) {
      writeStorage(CAMERA_TOPIC_KEY, topic);
    }
  },
  setModeReconciled: (value) => {
    set((state) => ({ robot: { ...state.robot, modeReconciled: value } }));
  },
  setNavKey: (key) => {
    set((state) => ({ ui: { ...state.ui, navKey: key } }));
    writeStorage(NAV_KEY, key);
  },
  toggleRightPanel: () => {
    set((state) => ({ ui: { ...state.ui, rightPanelOpen: !state.ui.rightPanelOpen } }));
  }
}));

export function getCachedMode(): RobotMode {
  return (readStorage(MODE_KEY, "manual") as RobotMode) || "manual";
}

export function getCachedCameraTopic(): string {
  return readStorage(CAMERA_TOPIC_KEY, "");
}
