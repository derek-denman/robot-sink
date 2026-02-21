const state = {
  latest: null,
  deadmanHeld: false,
  activeDrive: "stop",
  driveTimer: null,
};

async function api(path, method = "POST", body = undefined) {
  const response = await fetch(path, {
    method,
    headers: { "Content-Type": "application/json" },
    body: body ? JSON.stringify(body) : undefined,
  });

  const payload = await response.json().catch(() => ({}));
  if (!response.ok || payload.ok === false) {
    throw new Error(payload.error || payload.message || `${method} ${path} failed`);
  }
  return payload;
}

function toast(message, isError = false) {
  const el = document.getElementById("toast");
  el.textContent = message;
  el.style.background = isError ? "#7f1f1f" : "#1e2f3a";
  el.classList.add("show");
  window.setTimeout(() => el.classList.remove("show"), 2200);
}

function setText(id, text) {
  const el = document.getElementById(id);
  if (el) {
    el.textContent = text;
  }
}

function setStrongState(id, stateText) {
  const el = document.getElementById(id);
  if (!el) {
    return;
  }
  el.textContent = stateText;
  el.classList.remove("ok", "warn", "bad");
  if (/ok|armed|clear|connected|active|succeeded/i.test(stateText)) {
    el.classList.add("ok");
  } else if (/warn|idle|pending/i.test(stateText)) {
    el.classList.add("warn");
  } else if (/disarm|error|fail|estop|abort|canceled/i.test(stateText)) {
    el.classList.add("bad");
  }
}

function formatAge(unixTs) {
  if (!unixTs) return "n/a";
  const age = Math.max(0, Date.now() / 1000 - unixTs);
  return `${age.toFixed(1)}s ago`;
}

function formatTarget(target) {
  if (!target || target.x === null) {
    return "none";
  }
  return `x=${target.x}, y=${target.y}, z=${target.z}, conf=${target.confidence}`;
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

function renderStatus(snapshot) {
  state.latest = snapshot;

  const safety = snapshot.safety || {};
  const health = snapshot.health || {};
  const nav = snapshot.navigation || {};
  const dashboard = snapshot.dashboard || {};
  const mapping = snapshot.mapping || {};
  const pick = snapshot.pick_place || {};
  const demo = snapshot.demo || {};
  const recording = snapshot.recording || {};

  document.getElementById("modeSelect").value = snapshot.mode || "manual";

  setStrongState("safetyState", safety.armed ? "ARMED" : "DISARMED");
  setStrongState("estopState", safety.estop_latched ? "LATCHED" : "CLEAR");
  setStrongState("watchdogState", safety.watchdog_ok ? "OK" : "TRIPPED");
  setStrongState("baseState", health.base_connected ? "Connected" : "Disconnected");
  setStrongState("bagState", recording.status && recording.status.active ? "Recording" : "Stopped");
  setStrongState("healthState", health.tf_ok ? "OK" : "Needs Attention");

  setText("tileBase", health.base_connected ? "Connected" : "Disconnected");
  setText("tileEncoder", formatAge(dashboard.last_encoder_update_unix));
  setText("tileLidar", String((dashboard.topic_rates && dashboard.topic_rates.scan) || 0));
  setText("tileOak", String((dashboard.topic_rates && dashboard.topic_rates.camera) || 0));
  setText("tileNav", dashboard.nav_state || "idle");
  setText("tileTemps", (dashboard.temperatures || []).join(", ") || "n/a");

  setText("savedMapLabel", `Last map: ${mapping.last_saved_map || "none"}`);
  setText("navStateLabel", nav.state || "idle");
  setText("navResultLabel", nav.last_result || "none");

  setText("pickStage", pick.stage || "idle");
  setText("pickRetries", String(pick.retry_count || 0));
  setText("pickTarget", formatTarget(pick.last_target));

  setText("demoProgress", `${demo.completed_count || 0} / ${demo.target_count || 0}`);

  const reliability = snapshot.reliability || {};
  setText("reliabilityOutput", JSON.stringify(reliability, null, 2));

  const recordingsList = document.getElementById("recordingsList");
  recordingsList.innerHTML = "";
  ((recording.recent || []).slice(0, 8)).forEach((entry) => {
    const li = document.createElement("li");
    li.textContent = `${entry.name} (${Math.round((entry.size_bytes || 0) / 1024)} KiB)`;
    recordingsList.appendChild(li);
  });

  const checks = snapshot.commissioning || {};
  Object.keys(checks).forEach((checkId) => {
    const result = checks[checkId];
    const label = document.getElementById(`check-${checkId}`);
    if (!label) {
      return;
    }
    const prefix = result.passed ? "PASS" : "FAIL";
    label.textContent = `${prefix}: ${result.observed || ""}`;
  });

  updateFoxgloveLink(snapshot.foxglove && snapshot.foxglove.port);
}

function setTab(tabName) {
  document.querySelectorAll(".tab").forEach((el) => {
    el.classList.toggle("active", el.dataset.tab === tabName);
  });
  document.querySelectorAll(".panel").forEach((el) => {
    el.classList.toggle("active", el.id === `tab-${tabName}`);
  });
}

function driveVector(drive) {
  const speed = parseFloat(document.getElementById("speedSlider").value || "0.3");
  const turn = speed * 1.8;
  if (drive === "forward") return { linear_x: speed, angular_z: 0 };
  if (drive === "reverse") return { linear_x: -speed, angular_z: 0 };
  if (drive === "left") return { linear_x: 0, angular_z: turn };
  if (drive === "right") return { linear_x: 0, angular_z: -turn };
  return { linear_x: 0, angular_z: 0 };
}

async function sendDriveCommand(drive) {
  if (!state.deadmanHeld) {
    return;
  }
  const vec = driveVector(drive);
  await api("/api/teleop/cmd_vel", "POST", vec);
}

function startDriveLoop(drive) {
  state.activeDrive = drive;
  if (state.driveTimer) {
    window.clearInterval(state.driveTimer);
  }
  state.driveTimer = window.setInterval(() => {
    sendDriveCommand(state.activeDrive).catch((err) => toast(err.message, true));
  }, 140);
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

async function refreshStatus() {
  const status = await fetch("/api/status").then((r) => r.json());
  renderStatus(status);
}

function bindEvents() {
  document.querySelectorAll(".tab").forEach((el) => {
    el.addEventListener("click", () => setTab(el.dataset.tab));
  });

  document.getElementById("setModeBtn").addEventListener("click", async () => {
    const mode = document.getElementById("modeSelect").value;
    await api("/api/mode", "POST", { mode });
    toast(`Mode set: ${mode}`);
  });

  document.getElementById("armBtn").addEventListener("click", async () => {
    await api("/api/safety/arm", "POST", {});
    toast("Robot armed");
  });
  document.getElementById("disarmBtn").addEventListener("click", async () => {
    await api("/api/safety/disarm", "POST", {});
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
    toast("Stop-all dispatched", true);
  });

  document.getElementById("stackStartBtn").addEventListener("click", async () => {
    const out = await api("/api/stack/start", "POST", {});
    toast(out.result && out.result.ok ? "Stack start command sent" : "Stack start command failed", !(out.result && out.result.ok));
  });
  document.getElementById("stackStopBtn").addEventListener("click", async () => {
    const out = await api("/api/stack/stop", "POST", {});
    toast(out.result && out.result.ok ? "Stack stop command sent" : "Stack stop command failed", !(out.result && out.result.ok));
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
  document.getElementById("refreshRecordingsBtn").addEventListener("click", refreshStatus);

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
    toast(out.result && out.result.ok ? `Map saved: ${filename}` : "Save map failed", !(out.result && out.result.ok));
  });
  document.getElementById("switchLocalizationBtn").addEventListener("click", async () => {
    const out = await api("/api/mapping/switch_localization", "POST", {});
    toast(out.result && out.result.ok ? "Localization switch command sent" : "Localization switch not configured", !(out.result && out.result.ok));
  });
  document.getElementById("setInitialPoseBtn").addEventListener("click", async () => {
    const x = parseFloat(document.getElementById("initX").value || "0");
    const y = parseFloat(document.getElementById("initY").value || "0");
    const yaw = parseFloat(document.getElementById("initYaw").value || "0");
    await api("/api/localization/set_initial_pose", "POST", { x, y, yaw });
    toast("Initial pose published");
  });

  document.getElementById("sendGoalBtn").addEventListener("click", async () => {
    const x = parseFloat(document.getElementById("goalX").value || "0");
    const y = parseFloat(document.getElementById("goalY").value || "0");
    const yaw = parseFloat(document.getElementById("goalYaw").value || "0");
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

  document.getElementById("gripperOpenBtn").addEventListener("click", () => api("/api/arm/gripper", "POST", { command: "open" }).then(() => toast("Gripper open")));
  document.getElementById("gripperCloseBtn").addEventListener("click", () => api("/api/arm/gripper", "POST", { command: "close" }).then(() => toast("Gripper close")));
  document.getElementById("armStopBtn").addEventListener("click", () => api("/api/arm/stop", "POST", {}).then(() => toast("Arm stop sent", true)));
  document.getElementById("armPoseBtn").addEventListener("click", () => {
    const pose = document.getElementById("armPoseSelect").value;
    api("/api/arm/pose", "POST", { pose }).then(() => toast(`Pose: ${pose}`));
  });
  document.getElementById("jointJogBtn").addEventListener("click", () => {
    const joint = document.getElementById("jointNameInput").value;
    const delta = parseFloat(document.getElementById("jointDeltaInput").value || "0.05");
    api("/api/arm/jog", "POST", { joint, delta }).then(() => toast(`Jogged ${joint}`));
  });

  document.getElementById("pickStartBtn").addEventListener("click", () => api("/api/tasks/pick_place/start", "POST", {}).then(() => toast("Pick/place loop started")));
  document.getElementById("pickStopBtn").addEventListener("click", () => api("/api/tasks/pick_place/stop", "POST", {}).then(() => toast("Pick/place loop stopped")));
  document.getElementById("pickSkipBtn").addEventListener("click", () => api("/api/tasks/pick_place/skip", "POST", {}).then(() => toast("Skip requested")));
  document.getElementById("pickRetryBtn").addEventListener("click", () => api("/api/tasks/pick_place/retry", "POST", {}).then(() => toast("Retry requested")));
  document.getElementById("pickAbortBtn").addEventListener("click", () => api("/api/tasks/pick_place/abort", "POST", {}).then(() => toast("Pick/place aborted", true)));

  document.getElementById("demoRunBtn").addEventListener("click", async () => {
    const count = parseInt(document.getElementById("demoCountInput").value || "1", 10);
    await api("/api/demo/run", "POST", { count });
    toast(`Demo started: ${count} toys`);
  });
  document.getElementById("demoStopBtn").addEventListener("click", async () => {
    await api("/api/demo/stop", "POST", {});
    toast("Demo stopped", true);
  });

  const speedSlider = document.getElementById("speedSlider");
  speedSlider.addEventListener("input", () => {
    setText("speedLabel", `${parseFloat(speedSlider.value).toFixed(2)} m/s`);
  });

  const deadman = document.getElementById("deadmanBtn");
  deadman.addEventListener("mousedown", async () => {
    state.deadmanHeld = true;
    await api("/api/safety/arm", "POST", {});
    deadman.classList.add("accent");
  });
  deadman.addEventListener("mouseup", async () => {
    state.deadmanHeld = false;
    stopDriveLoop(true);
    await api("/api/safety/disarm", "POST", {});
    deadman.classList.remove("accent");
  });
  deadman.addEventListener("mouseleave", async () => {
    if (!state.deadmanHeld) return;
    state.deadmanHeld = false;
    stopDriveLoop(true);
    await api("/api/safety/disarm", "POST", {});
    deadman.classList.remove("accent");
  });

  document.querySelectorAll("[data-drive]").forEach((button) => {
    const drive = button.dataset.drive;
    button.addEventListener("mousedown", () => startDriveLoop(drive));
    button.addEventListener("mouseup", () => stopDriveLoop(true));
    button.addEventListener("mouseleave", () => stopDriveLoop(true));
  });

  window.addEventListener("keydown", (event) => {
    const keyMap = { w: "forward", a: "left", s: "reverse", d: "right" };
    const drive = keyMap[event.key.toLowerCase()];
    if (!drive) return;
    startDriveLoop(drive);
  });

  window.addEventListener("keyup", (event) => {
    if (["w", "a", "s", "d"].includes(event.key.toLowerCase())) {
      stopDriveLoop(true);
    }
  });
}

function connectWebSocket() {
  const protocol = window.location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${protocol}://${window.location.host}/ws`);

  ws.onmessage = (event) => {
    try {
      const msg = JSON.parse(event.data);
      if (msg.type === "status") {
        renderStatus(msg.data);
      }
    } catch (_) {
      // Ignore malformed payloads.
    }
  };

  ws.onclose = () => {
    window.setTimeout(connectWebSocket, 1800);
  };
}

async function main() {
  bindEvents();
  connectWebSocket();
  try {
    await refreshStatus();
  } catch (err) {
    toast(`Initial status failed: ${err.message}`, true);
  }

  window.setInterval(() => {
    if (!state.latest) {
      refreshStatus().catch(() => {});
    }
  }, 5000);
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
