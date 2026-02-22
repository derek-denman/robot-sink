const MODE_STORAGE_KEY = "robot_console.mode";
const TAB_STORAGE_KEY = "robot_console.tab";
const CAMERA_TOPIC_STORAGE_KEY = "robot_console.camera_topic";

const state = {
  latest: null,
  ws: null,
  wsConnected: false,
  wsReconnectTimer: null,
  reconcilePending: true,
  reconcileInFlight: false,
  modeFromBackend: "manual",
  modeApplyInFlight: false,
  pendingMode: "",
  lastStatusUnixMs: 0,

  // Motion control
  motionEnabledUntilMs: 0,
  motionArmRefreshTimer: null,
  deadmanDisarmSent: false,
  activeDrive: "stop",
  driveTimer: null,

  // Motor bank
  motorBankSendTimer: null,

  // Scan visualizer
  scanPayload: null,

  // Camera stream
  cameraTopic: "",
  cameraApplyInFlight: false,
  pendingCameraTopic: "",
  cameraFrameCount: 0,
  cameraFps: 0,
  cameraFpsWindowStartMs: 0,
  cameraFpsTimer: null,
};

function nowMs() {
  return Date.now();
}

function readLocalMode() {
  return (window.localStorage.getItem(MODE_STORAGE_KEY) || "").trim().toLowerCase();
}

function writeLocalMode(mode) {
  window.localStorage.setItem(MODE_STORAGE_KEY, String(mode || "manual"));
}

function readLocalTab() {
  return (window.localStorage.getItem(TAB_STORAGE_KEY) || "dashboard").trim();
}

function writeLocalTab(tab) {
  window.localStorage.setItem(TAB_STORAGE_KEY, tab);
}

function readLocalCameraTopic() {
  return (window.localStorage.getItem(CAMERA_TOPIC_STORAGE_KEY) || "").trim();
}

function writeLocalCameraTopic(topic) {
  window.localStorage.setItem(CAMERA_TOPIC_STORAGE_KEY, String(topic || ""));
}

async function fetchJson(path) {
  const response = await fetch(path, {
    method: "GET",
    headers: { Accept: "application/json" },
  });
  const payload = await response.json().catch(() => ({}));
  if (!response.ok) {
    throw new Error(payload.error || `GET ${path} failed (${response.status})`);
  }
  return payload;
}

async function api(path, method = "POST", body = undefined) {
  const response = await fetch(path, {
    method,
    headers: { "Content-Type": "application/json" },
    body: body ? JSON.stringify(body) : undefined,
  });

  const payload = await response.json().catch(() => ({}));
  if (!response.ok || payload.ok === false) {
    const nestedResult = payload && typeof payload.result === "object" ? payload.result : {};
    const nestedResults = nestedResult && Array.isArray(nestedResult.results) ? nestedResult.results : [];
    const nestedError = nestedResult.error || nestedResult.message;
    const listError = nestedResults.find((entry) => entry && entry.error);
    const derivedError = listError ? `${listError.service || "service"}: ${listError.error}` : "";
    const rawMessage =
      payload.error ||
      payload.message ||
      nestedError ||
      derivedError ||
      `${method} ${path} failed (${response.status})`;

    const friendlyMap = {
      manual_mode_required: "Switch to Manual mode before sending motion commands",
      robot_not_armed: "Robot is disarmed. Arm motion first.",
      navigate_to_pose_server_unavailable: "Nav2 NavigateToPose action server is unavailable",
      nav2_msgs_not_available: "nav2_msgs is missing in this ROS environment",
      no_active_goal: "No active navigation goal to cancel",
      service_unavailable: "Required ROS service is unavailable",
      slam_save_service_unavailable: "SLAM save-map service is unavailable",
      switch_to_localization_cmd_not_configured: "Localization switch command is not configured",
      command_not_configured: "Command is not configured",
    };

    throw new Error(friendlyMap[rawMessage] || rawMessage.replaceAll("_", " "));
  }
  return payload;
}

function toast(message, isError = false) {
  const el = document.getElementById("toast");
  el.textContent = message;
  el.style.background = isError ? "#7f1f1f" : "#17364c";
  el.classList.add("show");
  window.setTimeout(() => el.classList.remove("show"), 2400);
}

function setText(id, text) {
  const el = document.getElementById(id);
  if (el) {
    el.textContent = text;
  }
}

function formatAge(unixTs) {
  if (!unixTs) {
    return "n/a";
  }
  const ageSec = Math.max(0, nowMs() / 1000 - Number(unixTs));
  return `${ageSec.toFixed(1)}s ago`;
}

function formatHz(value) {
  const hz = Number(value || 0);
  return `${hz.toFixed(1)} Hz`;
}

function setSignalState(id, text) {
  const el = document.getElementById(id);
  if (!el) {
    return;
  }

  el.textContent = text;
  el.classList.remove("ok", "warn", "bad");
  const raw = String(text || "").toLowerCase();

  if (
    raw === "armed" ||
    raw === "connected" ||
    raw === "clear" ||
    raw === "ok" ||
    raw === "recording" ||
    raw === "active" ||
    raw === "succeeded" ||
    raw === "enabled" ||
    raw === "live"
  ) {
    el.classList.add("ok");
    return;
  }

  if (
    raw.includes("warn") ||
    raw.includes("idle") ||
    raw.includes("pending") ||
    raw.includes("connecting") ||
    raw.includes("needs")
  ) {
    el.classList.add("warn");
    return;
  }

  if (
    raw.includes("disarm") ||
    raw.includes("latched") ||
    raw.includes("tripped") ||
    raw.includes("disconnect") ||
    raw.includes("fail") ||
    raw.includes("error")
  ) {
    el.classList.add("bad");
    return;
  }

  el.classList.add("bad");
}

function updateFoxgloveLink(port) {
  const host = window.location.hostname || "<jetson>";
  const wsUrl = `ws://${host}:${port || 8765}`;
  setText("foxgloveWs", wsUrl);
  const webUrl = `https://studio.foxglove.dev/?ds=foxglove-websocket&ds.url=${encodeURIComponent(wsUrl)}`;
  const link = document.getElementById("foxgloveWebLink");
  if (link) {
    link.href = webUrl;
  }
}

function setTab(tabName) {
  document.querySelectorAll(".tab").forEach((el) => {
    el.classList.toggle("active", el.dataset.tab === tabName);
  });
  document.querySelectorAll(".panel").forEach((el) => {
    el.classList.toggle("active", el.id === `tab-${tabName}`);
  });
  writeLocalTab(tabName);
}

function drawScanCanvas(payload) {
  const canvas = document.getElementById("scanCanvas");
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

  ctx.fillStyle = "#0f1b25";
  ctx.fillRect(0, 0, width, height);

  const rangeMax = Number(payload && payload.range_max ? payload.range_max : 6.0);
  const plotRange = Math.max(1.0, Math.min(20.0, rangeMax));
  const scale = Math.min(width, height) * 0.45 / plotRange;

  ctx.strokeStyle = "rgba(120, 168, 202, 0.24)";
  ctx.lineWidth = 1;
  for (let ring = 1; ring <= 4; ring += 1) {
    ctx.beginPath();
    ctx.arc(cx, cy, ring * (Math.min(width, height) * 0.1), 0, Math.PI * 2);
    ctx.stroke();
  }

  ctx.strokeStyle = "rgba(196, 218, 236, 0.32)";
  ctx.beginPath();
  ctx.moveTo(cx, 0);
  ctx.lineTo(cx, height);
  ctx.moveTo(0, cy);
  ctx.lineTo(width, cy);
  ctx.stroke();

  const points = Array.isArray(payload && payload.points) ? payload.points : [];
  ctx.fillStyle = "#57d0ff";
  for (let i = 0; i < points.length; i += 1) {
    const point = points[i];
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
    if (px < 0 || py < 0 || px >= width || py >= height) {
      continue;
    }

    ctx.fillRect(px, py, 2, 2);
  }

  ctx.fillStyle = "rgba(226, 240, 249, 0.9)";
  ctx.font = "12px 'Menlo', 'Consolas', monospace";
  ctx.fillText(`pts: ${points.length}`, 10, 18);
  ctx.fillText(`frame: ${payload && payload.frame_id ? payload.frame_id : 'n/a'}`, 10, 34);
}

function updateScanStatus(snapshot) {
  const scan = (snapshot.visualizer && snapshot.visualizer.scan) || {};
  setSignalState("scanStatus", scan.connected ? "Connected" : "Disconnected");
  setText("scanFps", formatHz(scan.fps));
  if (scan.age_sec === null || scan.age_sec === undefined) {
    setText("scanAge", "age n/a");
  } else {
    setText("scanAge", `age ${Number(scan.age_sec).toFixed(2)}s`);
  }
}

function updateCameraStatus(snapshot) {
  const camera = (snapshot.visualizer && snapshot.visualizer.camera) || {};
  setSignalState("cameraStatus", camera.connected ? "Connected" : "Disconnected");
  setText("cameraFps", formatHz(camera.fps));
  if (camera.age_sec === null || camera.age_sec === undefined) {
    setText("cameraAge", "age n/a");
  } else {
    setText("cameraAge", `age ${Number(camera.age_sec).toFixed(2)}s`);
  }

  const selectedTopic = String(camera.selected_topic || "");
  if (state.pendingCameraTopic && selectedTopic === state.pendingCameraTopic) {
    state.pendingCameraTopic = "";
  }
  const displayTopic = state.pendingCameraTopic || selectedTopic;
  setText("cameraTopicLabel", `topic: ${displayTopic || "n/a"}`);
  populateCameraTopics(camera.available_topics || [], displayTopic);

  if (displayTopic && displayTopic !== state.cameraTopic) {
    state.cameraTopic = displayTopic;
    writeLocalCameraTopic(displayTopic);
    startCameraStream();
  }
}

function populateCameraTopics(topics, selectedTopic) {
  const select = document.getElementById("cameraTopicSelect");
  if (!select) {
    return;
  }

  const normalized = Array.isArray(topics) ? topics : [];
  const currentOptions = new Set(Array.from(select.options).map((opt) => opt.value));
  const needsRewrite =
    normalized.length !== currentOptions.size ||
    normalized.some((topic) => !currentOptions.has(topic));

  if (needsRewrite) {
    select.innerHTML = "";
    normalized.forEach((topic) => {
      const option = document.createElement("option");
      option.value = topic;
      option.textContent = topic;
      select.appendChild(option);
    });
  }

  if (selectedTopic) {
    select.value = selectedTopic;
  }
}

async function applyCameraTopicSelection(topic, showToast = true) {
  const selectedTopic = String(topic || "").trim();
  if (!selectedTopic) {
    return;
  }

  if (state.cameraApplyInFlight && state.pendingCameraTopic === selectedTopic) {
    return;
  }

  state.pendingCameraTopic = selectedTopic;
  state.cameraApplyInFlight = true;
  writeLocalCameraTopic(selectedTopic);

  try {
    await api("/api/camera/select", "POST", { topic: selectedTopic });
    state.cameraTopic = selectedTopic;
    startCameraStream();
    if (showToast) {
      toast(`Camera topic selected: ${selectedTopic}`);
    }
    if (state.ws && state.wsConnected) {
      state.ws.send(JSON.stringify({ type: "camera_select", topic: selectedTopic }));
    }
  } catch (err) {
    state.pendingCameraTopic = "";
    throw err;
  } finally {
    state.cameraApplyInFlight = false;
  }
}

function startCameraFpsCounter() {
  if (state.cameraFpsTimer) {
    window.clearInterval(state.cameraFpsTimer);
    state.cameraFpsTimer = null;
  }

  state.cameraFrameCount = 0;
  state.cameraFps = 0;
  state.cameraFpsWindowStartMs = nowMs();

  state.cameraFpsTimer = window.setInterval(() => {
    const elapsed = Math.max(1, nowMs() - state.cameraFpsWindowStartMs) / 1000;
    state.cameraFps = state.cameraFrameCount / elapsed;
    state.cameraFrameCount = 0;
    state.cameraFpsWindowStartMs = nowMs();
  }, 1200);
}

function startCameraStream() {
  const img = document.getElementById("cameraStreamImg");
  if (!img) {
    return;
  }

  if (!state.cameraTopic) {
    img.removeAttribute("src");
    return;
  }

  const query = new URLSearchParams({
    topic: state.cameraTopic,
    t: String(Date.now()),
  });
  img.src = `/stream/camera.mjpeg?${query.toString()}`;
}

function ensureMotionEnabledForCommand() {
  const enabled = nowMs() < state.motionEnabledUntilMs;
  if (enabled) {
    return true;
  }
  toast("Enable motion first", true);
  return false;
}

function updateDeadmanUi() {
  const enabled = nowMs() < state.motionEnabledUntilMs;
  setSignalState("deadmanState", enabled ? "enabled" : "disabled");
  const remaining = enabled ? (state.motionEnabledUntilMs - nowMs()) / 1000 : 0;
  setText("deadmanCountdown", `${Math.max(0, remaining).toFixed(1)}s`);

  if (!enabled && !state.deadmanDisarmSent) {
    state.deadmanDisarmSent = true;
    stopDriveLoop(true);
    sendMotorBank(0, 0).catch(() => {});
    api("/api/arm", "POST", { armed: false }).catch(() => {});
  }
}

async function enableMotionWindow() {
  await api("/api/arm", "POST", { armed: true });
  state.motionEnabledUntilMs = nowMs() + 5000;
  state.deadmanDisarmSent = false;
  updateDeadmanUi();
}

function driveVector(drive) {
  const speed = Number(document.getElementById("speedSlider").value || 0.3);
  const turn = speed * 1.8;
  if (drive === "forward") return { linear_x: speed, angular_z: 0 };
  if (drive === "reverse") return { linear_x: -speed, angular_z: 0 };
  if (drive === "left") return { linear_x: 0, angular_z: turn };
  if (drive === "right") return { linear_x: 0, angular_z: -turn };
  return { linear_x: 0, angular_z: 0 };
}

async function sendDriveCommand(drive) {
  if (!ensureMotionEnabledForCommand()) {
    return;
  }

  const vector = driveVector(drive);
  await api("/api/cmd_vel", "POST", vector);

  if (drive !== "stop") {
    state.motionEnabledUntilMs = nowMs() + 5000;
    state.deadmanDisarmSent = false;
  }

  updateDeadmanUi();
}

function startDriveLoop(drive) {
  state.activeDrive = drive;
  if (state.driveTimer) {
    window.clearInterval(state.driveTimer);
    state.driveTimer = null;
  }

  if (drive === "stop") {
    sendDriveCommand("stop").catch((err) => toast(err.message, true));
    return;
  }

  state.driveTimer = window.setInterval(() => {
    sendDriveCommand(state.activeDrive).catch((err) => toast(err.message, true));
  }, 130);
}

function stopDriveLoop(sendStop = true) {
  state.activeDrive = "stop";
  if (state.driveTimer) {
    window.clearInterval(state.driveTimer);
    state.driveTimer = null;
  }

  if (sendStop) {
    sendDriveCommand("stop").catch(() => {});
  }
}

function readMotorBankValues() {
  const left = Number(document.getElementById("leftBankSlider").value || 0);
  const right = Number(document.getElementById("rightBankSlider").value || 0);
  return { left, right };
}

function renderMotorBankLabels() {
  const values = readMotorBankValues();
  setText("leftBankValue", String(values.left));
  setText("rightBankValue", String(values.right));
}

async function sendMotorBank(left, right) {
  if (!ensureMotionEnabledForCommand()) {
    return;
  }

  await api("/api/motor_bank", "POST", { left, right });
  if (left !== 0 || right !== 0) {
    state.motionEnabledUntilMs = nowMs() + 5000;
    state.deadmanDisarmSent = false;
  }
  updateDeadmanUi();
}

function scheduleMotorBankSend() {
  if (state.motorBankSendTimer) {
    return;
  }

  state.motorBankSendTimer = window.setTimeout(() => {
    state.motorBankSendTimer = null;
    const values = readMotorBankValues();
    sendMotorBank(values.left, values.right).catch(() => {});
  }, 160);
}

function currentJointsPayload() {
  const jointNames = ["j1", "j2", "j3", "j4", "j5"];
  const joints = {};
  jointNames.forEach((name) => {
    const value = Number(document.getElementById(`joint-${name}`).value || 0);
    joints[name] = value;
  });
  return joints;
}

function renderJointLabels() {
  ["j1", "j2", "j3", "j4", "j5"].forEach((name) => {
    const value = Number(document.getElementById(`joint-${name}`).value || 0);
    setText(`joint-${name}-val`, value.toFixed(2));
  });
}

function updateModeUi(mode) {
  const select = document.getElementById("modeSelect");
  if (select && select.value !== mode) {
    select.value = mode;
  }
  setText("modeState", mode);
}

async function applyModeSelection(mode, showToast = true) {
  const selectedMode = String(mode || "").trim().toLowerCase();
  if (!selectedMode) {
    return;
  }

  if (state.modeApplyInFlight && state.pendingMode === selectedMode) {
    return;
  }

  state.pendingMode = selectedMode;
  state.modeApplyInFlight = true;
  writeLocalMode(selectedMode);
  updateModeUi(selectedMode);

  try {
    await api("/api/mode", "POST", { mode: selectedMode });
    if (showToast) {
      toast(`Mode set: ${selectedMode}`);
    }
  } catch (err) {
    state.pendingMode = "";
    updateModeUi(state.modeFromBackend || "manual");
    writeLocalMode(state.modeFromBackend || "manual");
    throw err;
  } finally {
    state.modeApplyInFlight = false;
  }
}

async function reconcileModeFromLocalStorage(snapshotMode) {
  if (state.reconcileInFlight) {
    return;
  }

  const localMode = readLocalMode();
  if (!localMode) {
    writeLocalMode(snapshotMode);
    state.reconcilePending = false;
    return;
  }

  if (localMode === snapshotMode) {
    state.reconcilePending = false;
    return;
  }

  state.reconcileInFlight = true;
  try {
    await applyModeSelection(localMode, false);
    toast(`Mode reconciled from local preference: ${localMode}`);
  } catch (err) {
    state.pendingMode = "";
    writeLocalMode(snapshotMode);
    toast(`Mode reconcile fallback to backend mode: ${snapshotMode}`, true);
  } finally {
    state.reconcilePending = false;
    state.reconcileInFlight = false;
  }
}

function renderPickTarget(target) {
  if (!target || target.x === null || target.x === undefined) {
    return "none";
  }
  return `x=${target.x}, y=${target.y}, z=${target.z}, conf=${target.confidence}`;
}

function renderStatus(snapshot) {
  state.latest = snapshot;
  state.lastStatusUnixMs = nowMs();
  setSignalState("lastUpdateState", "live");

  const safety = snapshot.safety || {};
  const health = snapshot.health || {};
  const nav = snapshot.navigation || {};
  const dashboard = snapshot.dashboard || {};
  const mapping = snapshot.mapping || {};
  const pick = snapshot.pick_place || {};
  const demo = snapshot.demo || {};
  const recording = snapshot.recording || {};
  const reliability = snapshot.reliability || {};

  const backendMode = String(snapshot.mode || "manual");
  state.modeFromBackend = backendMode;
  if (state.pendingMode && backendMode === state.pendingMode) {
    state.pendingMode = "";
  }
  updateModeUi(state.pendingMode || backendMode);
  if (!state.reconcilePending && !state.pendingMode) {
    writeLocalMode(backendMode);
  }

  setSignalState("safetyState", safety.armed ? "ARMED" : "DISARMED");
  setSignalState("estopState", safety.estop_latched ? "LATCHED" : "CLEAR");
  setSignalState("watchdogState", safety.watchdog_ok ? "OK" : "TRIPPED");
  setSignalState("baseState", health.base_connected ? "Connected" : "Disconnected");
  setSignalState("bagState", recording.status && recording.status.active ? "Recording" : "Stopped");
  setSignalState("healthState", health.tf_ok ? "OK" : "Needs Attention");

  setText("tileBase", health.base_connected ? "Connected" : "Disconnected");
  setText("tileEncoder", formatAge(dashboard.last_encoder_update_unix));
  setText("tileLidar", formatHz(dashboard.topic_rates && dashboard.topic_rates.scan));
  const visualizerCameraHz = snapshot.visualizer && snapshot.visualizer.camera ? Number(snapshot.visualizer.camera.fps || 0) : 0;
  const topicCameraHz = dashboard.topic_rates ? Number(dashboard.topic_rates.camera || 0) : 0;
  setText("tileOak", formatHz(Math.max(topicCameraHz, visualizerCameraHz)));
  setText("tileNav", dashboard.nav_state || "idle");
  setText("tileTemps", (dashboard.temperatures || []).join(", ") || "n/a");

  setText("savedMapLabel", `Last map: ${mapping.last_saved_map || "none"}`);
  setText("navStateLabel", nav.state || "idle");
  setText("navResultLabel", nav.last_result || "none");

  setText("pickStage", pick.stage || "idle");
  setText("pickRetries", String(pick.retry_count || 0));
  setText("pickTarget", renderPickTarget(pick.last_target));

  setText("demoProgress", `${demo.completed_count || 0} / ${demo.target_count || 0}`);

  setText("reliabilityOutput", JSON.stringify(reliability, null, 2));

  const recordingsList = document.getElementById("recordingsList");
  recordingsList.innerHTML = "";
  (recording.recent || []).slice(0, 10).forEach((entry) => {
    const item = document.createElement("li");
    const sizeKb = Math.round((entry.size_bytes || 0) / 1024);
    item.textContent = `${entry.name} (${sizeKb} KiB)`;
    recordingsList.appendChild(item);
  });

  const checks = snapshot.commissioning || {};
  Object.keys(checks).forEach((checkId) => {
    const result = checks[checkId];
    const label = document.getElementById(`check-${checkId}`);
    if (!label) {
      return;
    }
    label.textContent = `${result.passed ? "PASS" : "FAIL"}: ${result.observed || ""}`;
  });

  updateScanStatus(snapshot);
  updateCameraStatus(snapshot);
  updateFoxgloveLink(snapshot.foxglove && snapshot.foxglove.port);

  if (state.reconcilePending) {
    reconcileModeFromLocalStorage(backendMode).catch(() => {});
  }
}

function handleScanMessage(payload) {
  if (!payload) {
    return;
  }
  state.scanPayload = payload;
  drawScanCanvas(payload);
}

function updateWsState() {
  if (state.wsConnected) {
    setSignalState("wsState", "connected");
    return;
  }
  setSignalState("wsState", "disconnected");
}

function markDisconnectedAge() {
  if (state.wsConnected || !state.lastStatusUnixMs) {
    return;
  }
  const age = Math.max(0, (nowMs() - state.lastStatusUnixMs) / 1000);
  setSignalState("lastUpdateState", `${age.toFixed(1)}s old`);
}

async function refreshStatus() {
  const status = await fetchJson("/api/status");
  renderStatus(status);
  if (state.scanPayload == null) {
    const scanSnapshot = await fetchJson("/api/scan").catch(() => null);
    if (scanSnapshot && scanSnapshot.scan) {
      handleScanMessage(scanSnapshot.scan);
    }
  }
}

async function refreshCameraTopics() {
  const out = await fetchJson("/api/camera/topics");
  populateCameraTopics(out.topics || [], out.selected_topic || "");
  if (out.selected_topic && out.selected_topic !== state.cameraTopic) {
    state.cameraTopic = out.selected_topic;
    writeLocalCameraTopic(out.selected_topic);
    startCameraStream();
  }

  const preferredTopic = readLocalCameraTopic();
  if (preferredTopic && preferredTopic !== out.selected_topic && (out.topics || []).includes(preferredTopic)) {
    applyCameraTopicSelection(preferredTopic, false).catch(() => {});
  }
}

function connectWebSocket() {
  if (state.ws) {
    try {
      state.ws.close();
    } catch (_) {
      // no-op
    }
  }

  const protocol = window.location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${protocol}://${window.location.host}/ws`);
  state.ws = ws;

  ws.onopen = () => {
    state.wsConnected = true;
    state.reconcilePending = true;
    state.pendingMode = readLocalMode() || state.pendingMode;
    updateWsState();
  };

  ws.onmessage = (event) => {
    try {
      const msg = JSON.parse(event.data);
      if (msg.type === "status") {
        renderStatus(msg.data || {});
      } else if (msg.type === "scan") {
        handleScanMessage(msg.data || null);
      }
    } catch (_) {
      // ignore malformed payload
    }
  };

  ws.onerror = () => {
    state.wsConnected = false;
    updateWsState();
  };

  ws.onclose = () => {
    state.wsConnected = false;
    updateWsState();
    if (state.wsReconnectTimer) {
      window.clearTimeout(state.wsReconnectTimer);
    }
    state.wsReconnectTimer = window.setTimeout(connectWebSocket, 1400);
  };
}

function bindManualControls() {
  const speedSlider = document.getElementById("speedSlider");
  speedSlider.addEventListener("input", () => {
    setText("speedLabel", `${Number(speedSlider.value).toFixed(2)} m/s`);
  });

  const deadmanBtn = document.getElementById("deadmanToggleBtn");
  deadmanBtn.addEventListener("click", async () => {
    try {
      await enableMotionWindow();
      toast("Motion enabled for 5 seconds");
    } catch (err) {
      toast(err.message, true);
    }
  });

  document.querySelectorAll("[data-drive]").forEach((button) => {
    const drive = button.dataset.drive;
    const onDown = (event) => {
      event.preventDefault();
      if (drive === "stop") {
        stopDriveLoop(true);
        return;
      }
      startDriveLoop(drive);
    };

    const onUp = () => stopDriveLoop(true);

    button.addEventListener("mousedown", onDown);
    button.addEventListener("touchstart", onDown, { passive: false });
    button.addEventListener("mouseup", onUp);
    button.addEventListener("mouseleave", onUp);
    button.addEventListener("touchend", onUp);
    button.addEventListener("touchcancel", onUp);
  });

  const leftSlider = document.getElementById("leftBankSlider");
  const rightSlider = document.getElementById("rightBankSlider");

  const onSlider = () => {
    renderMotorBankLabels();
    scheduleMotorBankSend();
  };
  leftSlider.addEventListener("input", onSlider);
  rightSlider.addEventListener("input", onSlider);

  document.getElementById("sendMotorBankBtn").addEventListener("click", async () => {
    try {
      const values = readMotorBankValues();
      await sendMotorBank(values.left, values.right);
      toast(`Motor bank L=${values.left} R=${values.right}`);
    } catch (err) {
      toast(err.message, true);
    }
  });

  document.getElementById("centerMotorBankBtn").addEventListener("click", async () => {
    leftSlider.value = "0";
    rightSlider.value = "0";
    renderMotorBankLabels();
    try {
      await sendMotorBank(0, 0);
      toast("Motor bank centered and stop sent");
    } catch (err) {
      toast(err.message, true);
    }
  });

  ["j1", "j2", "j3", "j4", "j5"].forEach((name) => {
    document.getElementById(`joint-${name}`).addEventListener("input", renderJointLabels);
  });

  document.getElementById("sendJointsBtn").addEventListener("click", async () => {
    const joints = currentJointsPayload();
    await api("/api/arm/joints", "POST", { joints });
    toast("Joint command sent");
  });

  document.getElementById("armStopBtn").addEventListener("click", async () => {
    await api("/api/arm/stop", "POST", {});
    toast("Arm stop sent", true);
  });

  document.querySelectorAll(".pose-btn").forEach((button) => {
    button.addEventListener("click", async () => {
      const pose = button.dataset.pose;
      await api("/api/arm/pose", "POST", { pose });
      toast(`Pose command: ${pose}`);
    });
  });

  document.getElementById("gripperOpenBtn").addEventListener("click", async () => {
    await api("/api/gripper", "POST", { command: "open" });
    toast("Gripper open");
  });

  document.getElementById("gripperCloseBtn").addEventListener("click", async () => {
    await api("/api/gripper", "POST", { command: "close" });
    toast("Gripper close");
  });
}

function bindGeneralEvents() {
  document.querySelectorAll(".tab").forEach((el) => {
    el.addEventListener("click", () => setTab(el.dataset.tab));
  });

  const modeSelect = document.getElementById("modeSelect");
  modeSelect.addEventListener("change", () => {
    const mode = modeSelect.value;
    applyModeSelection(mode, false).catch((err) => toast(err.message, true));
  });

  document.getElementById("setModeBtn").addEventListener("click", async () => {
    const mode = modeSelect.value;
    await applyModeSelection(mode, true);
  });

  document.getElementById("armBtn").addEventListener("click", async () => {
    await api("/api/arm", "POST", { armed: true });
    toast("Robot armed");
  });

  document.getElementById("disarmBtn").addEventListener("click", async () => {
    await api("/api/arm", "POST", { armed: false });
    toast("Robot disarmed");
  });

  document.getElementById("estopBtn").addEventListener("click", async () => {
    await api("/api/safety/estop", "POST", {});
    toast("E-stop latched", true);
  });

  document.getElementById("resetEstopBtn").addEventListener("click", async () => {
    await api("/api/safety/reset_estop", "POST", {});
    toast("E-stop reset");
  });

  document.getElementById("stopAllBtn").addEventListener("click", async () => {
    await api("/api/stop_all", "POST", {});
    state.motionEnabledUntilMs = 0;
    updateDeadmanUi();
    toast("Stop-all sent", true);
  });

  document.getElementById("cameraRefreshTopicsBtn").addEventListener("click", () => {
    refreshCameraTopics().catch((err) => toast(err.message, true));
  });

  const cameraTopicSelect = document.getElementById("cameraTopicSelect");
  cameraTopicSelect.addEventListener("change", () => {
    const topic = cameraTopicSelect.value;
    applyCameraTopicSelection(topic, false).catch((err) => toast(err.message, true));
  });

  document.getElementById("cameraSelectBtn").addEventListener("click", async () => {
    const topic = cameraTopicSelect.value;
    await applyCameraTopicSelection(topic, true);
  });

  document.getElementById("stackStartBtn").addEventListener("click", async () => {
    const out = await api("/api/stack/start", "POST", {});
    toast(out.result && out.result.ok ? "Stack start command sent" : "Stack start failed", !(out.result && out.result.ok));
  });

  document.getElementById("stackStopBtn").addEventListener("click", async () => {
    const out = await api("/api/stack/stop", "POST", {});
    toast(out.result && out.result.ok ? "Stack stop command sent" : "Stack stop failed", !(out.result && out.result.ok));
  });

  document.querySelectorAll("[data-check]").forEach((button) => {
    button.addEventListener("click", async () => {
      const check = button.dataset.check;
      const out = await api("/api/checklist/run", "POST", { check });
      const result = out.result || {};
      toast(`${check}: ${result.passed ? "PASS" : "FAIL"}`, !result.passed);
      await refreshStatus();
    });
  });

  document.getElementById("recordStartBtn").addEventListener("click", async () => {
    const tags = document.getElementById("recordTagInput").value || "dashboard";
    await api("/api/recording/start", "POST", { tags });
    toast("Recording started");
  });

  document.getElementById("recordStopBtn").addEventListener("click", async () => {
    await api("/api/recording/stop", "POST", {});
    toast("Recording stopped");
  });

  document.getElementById("trainRecordStartBtn").addEventListener("click", async () => {
    const tags = document.getElementById("trainTagsInput").value || "training";
    await api("/api/recording/start", "POST", { tags });
    toast("Training bag started");
  });

  document.getElementById("trainRecordStopBtn").addEventListener("click", async () => {
    await api("/api/recording/stop", "POST", {});
    toast("Training bag stopped");
  });

  document.getElementById("refreshRecordingsBtn").addEventListener("click", () => {
    refreshStatus().catch((err) => toast(err.message, true));
  });

  document.getElementById("replayHintBtn").addEventListener("click", async () => {
    const path = document.getElementById("replayPathInput").value;
    const out = await api("/api/recording/replay_hint", "POST", { path });
    setText("replayHintOutput", out.command || "");
  });

  document.getElementById("slamStartBtn").addEventListener("click", async () => {
    const out = await api("/api/mapping/start", "POST", {});
    toast(out.result && out.result.ok ? "SLAM started" : "SLAM start failed", !(out.result && out.result.ok));
  });

  document.getElementById("slamStopBtn").addEventListener("click", async () => {
    const out = await api("/api/mapping/stop", "POST", {});
    toast(out.result && out.result.ok ? "SLAM stopped" : "SLAM stop failed", !(out.result && out.result.ok));
  });

  document.getElementById("saveMapBtn").addEventListener("click", async () => {
    const filename = document.getElementById("mapNameInput").value || "map_snapshot";
    const out = await api("/api/mapping/save", "POST", { filename });
    toast(out.result && out.result.ok ? `Map saved: ${filename}` : "Map save failed", !(out.result && out.result.ok));
  });

  document.getElementById("switchLocalizationBtn").addEventListener("click", async () => {
    const out = await api("/api/mapping/switch_localization", "POST", {});
    toast(out.result && out.result.ok ? "Localization switched" : "Localization switch failed", !(out.result && out.result.ok));
  });

  document.getElementById("setInitialPoseBtn").addEventListener("click", async () => {
    const x = Number(document.getElementById("initX").value || 0);
    const y = Number(document.getElementById("initY").value || 0);
    const yaw = Number(document.getElementById("initYaw").value || 0);
    await api("/api/localization/set_initial_pose", "POST", { x, y, yaw });
    toast("Initial pose sent");
  });

  document.getElementById("sendGoalBtn").addEventListener("click", async () => {
    const x = Number(document.getElementById("goalX").value || 0);
    const y = Number(document.getElementById("goalY").value || 0);
    const yaw = Number(document.getElementById("goalYaw").value || 0);
    const out = await api("/api/nav/goal", "POST", { x, y, yaw });
    toast(out.result && out.result.ok ? "Navigation goal sent" : "Navigation goal failed", !(out.result && out.result.ok));
  });

  document.getElementById("cancelGoalBtn").addEventListener("click", async () => {
    await api("/api/nav/cancel", "POST", {});
    toast("Navigation cancel sent");
  });

  document.getElementById("clearCostmapsBtn").addEventListener("click", async () => {
    const out = await api("/api/nav/clear_costmaps", "POST", {});
    toast(out.result && out.result.ok ? "Costmaps cleared" : "Clear costmaps failed", !(out.result && out.result.ok));
  });

  document.getElementById("pickStartBtn").addEventListener("click", async () => {
    await api("/api/tasks/pick_place/start", "POST", {});
    toast("Pick/place loop started");
  });

  document.getElementById("pickStopBtn").addEventListener("click", async () => {
    await api("/api/tasks/pick_place/stop", "POST", {});
    toast("Pick/place loop stopped");
  });

  document.getElementById("pickSkipBtn").addEventListener("click", async () => {
    await api("/api/tasks/pick_place/skip", "POST", {});
    toast("Skip requested");
  });

  document.getElementById("pickRetryBtn").addEventListener("click", async () => {
    await api("/api/tasks/pick_place/retry", "POST", {});
    toast("Retry grasp requested");
  });

  document.getElementById("pickAbortBtn").addEventListener("click", async () => {
    await api("/api/tasks/pick_place/abort", "POST", {});
    toast("Pick/place aborted", true);
  });

  document.getElementById("demoRunBtn").addEventListener("click", async () => {
    const count = parseInt(document.getElementById("demoCountInput").value || "1", 10);
    await api("/api/demo/run", "POST", { count });
    toast(`Demo started for ${count} objects`);
  });

  document.getElementById("demoStopBtn").addEventListener("click", async () => {
    await api("/api/demo/stop", "POST", {});
    toast("Demo stopped", true);
  });
}

function bootLocalSelections() {
  const mode = readLocalMode();
  if (mode) {
    const select = document.getElementById("modeSelect");
    if (select) {
      select.value = mode;
    }
    state.pendingMode = mode;
  }

  const cameraTopic = readLocalCameraTopic();
  if (cameraTopic) {
    state.pendingCameraTopic = cameraTopic;
  }

  const tab = readLocalTab();
  setTab(tab || "dashboard");
}

async function main() {
  bootLocalSelections();
  bindGeneralEvents();
  bindManualControls();
  renderMotorBankLabels();
  renderJointLabels();
  drawScanCanvas({ points: [], range_max: 6.0, frame_id: "none" });
  startCameraFpsCounter();

  const streamImg = document.getElementById("cameraStreamImg");
  streamImg.addEventListener("load", () => {
    state.cameraFrameCount += 1;
  });

  streamImg.addEventListener("error", () => {
    setSignalState("cameraStatus", "Disconnected");
  });

  connectWebSocket();

  try {
    await refreshStatus();
  } catch (err) {
    toast(`Initial status failed: ${err.message}`, true);
  }

  refreshCameraTopics().catch(() => {});

  window.setInterval(() => {
    updateDeadmanUi();
    markDisconnectedAge();
  }, 100);

  window.setInterval(() => {
    if (!state.wsConnected) {
      refreshStatus().catch(() => {});
    }
  }, 3000);
}

window.addEventListener("load", () => {
  main().catch((err) => {
    toast(err.message, true);
  });
});

window.addEventListener("unhandledrejection", (event) => {
  const reason = event.reason;
  const message = reason && reason.message ? reason.message : "Unhandled request error";
  toast(message, true);
});
