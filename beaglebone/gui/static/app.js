const connectionEl = document.getElementById("connection");
const armedStateEl = document.getElementById("armedState");
const watchdogStateEl = document.getElementById("watchdogState");
const cmdAgeEl = document.getElementById("cmdAge");
const encAgeEl = document.getElementById("encAge");
const encoderRowsEl = document.getElementById("encoderRows");

const armBtn = document.getElementById("armBtn");
const estopBtn = document.getElementById("estopBtn");
const resetEncBtn = document.getElementById("resetEncBtn");

const leftSlider = document.getElementById("leftSlider");
const rightSlider = document.getElementById("rightSlider");
const leftValue = document.getElementById("leftValue");
const rightValue = document.getElementById("rightValue");

const linearSlider = document.getElementById("linearSlider");
const angularSlider = document.getElementById("angularSlider");
const linearValue = document.getElementById("linearValue");
const angularValue = document.getElementById("angularValue");

const tankControls = document.getElementById("tankControls");
const cmdVelControls = document.getElementById("cmdVelControls");

let latestStatus = null;
let ws = null;

function setConnection(ok) {
  connectionEl.textContent = ok ? "WS Online" : "WS Offline";
  connectionEl.classList.toggle("online", ok);
  connectionEl.classList.toggle("offline", !ok);
}

function renderStatus(status) {
  latestStatus = status;

  const armed = status.armed && !status.estop_asserted;
  armedStateEl.textContent = armed ? "Armed" : "Disarmed";
  armedStateEl.classList.toggle("armed", armed);
  armedStateEl.classList.toggle("disarmed", !armed);
  armBtn.textContent = armed ? "Disarm" : "Arm";
  estopBtn.textContent = status.estop_asserted ? "Release E-STOP" : "E-STOP";

  watchdogStateEl.textContent = status.watchdog_tripped ? "TRIPPED" : "OK";
  cmdAgeEl.textContent = `${status.last_motor_command_age_ms} ms`;
  encAgeEl.textContent = status.encoder.age_ms === null ? "-" : `${status.encoder.age_ms} ms`;

  encoderRowsEl.innerHTML = "";
  for (let i = 0; i < 4; i += 1) {
    const tr = document.createElement("tr");
    tr.innerHTML = `
      <td>${i}</td>
      <td>${status.encoder.counts[i]}</td>
      <td>${status.encoder.velocity_tps[i]}</td>
    `;
    encoderRowsEl.appendChild(tr);
  }
}

async function apiPost(path, payload = {}) {
  const response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  const data = await response.json();
  if (!response.ok) {
    throw new Error(data.error || data.reason || `HTTP ${response.status}`);
  }
  if (data.status) {
    renderStatus(data.status);
  }
  return data;
}

function currentDriveMode() {
  return document.querySelector('input[name="driveMode"]:checked').value;
}

function updateControlVisibility() {
  const mode = currentDriveMode();
  tankControls.classList.toggle("hidden", mode !== "tank");
  cmdVelControls.classList.toggle("hidden", mode !== "cmdvel");
}

function readDriveCommand() {
  if (currentDriveMode() === "tank") {
    return {
      left: parseInt(leftSlider.value, 10),
      right: parseInt(rightSlider.value, 10),
    };
  }

  return {
    linear: parseFloat(linearSlider.value),
    angular: parseFloat(angularSlider.value),
  };
}

let sendTimer = null;
function scheduleMotorPush() {
  if (sendTimer) {
    return;
  }

  sendTimer = window.setTimeout(async () => {
    sendTimer = null;
    if (!latestStatus || !latestStatus.armed || latestStatus.estop_asserted) {
      return;
    }
    try {
      await apiPost("/api/motor", readDriveCommand());
    } catch (err) {
      console.error(err);
    }
  }, 80);
}

async function pushZero() {
  try {
    await apiPost("/api/motor", { left: 0, right: 0 });
  } catch (err) {
    console.error(err);
  }
}

function bindSlider(slider, out, formatter = (v) => v) {
  slider.addEventListener("input", () => {
    out.textContent = formatter(slider.value);
    scheduleMotorPush();
  });
}

function connectWebSocket() {
  const wsProto = window.location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(`${wsProto}://${window.location.host}/ws`);

  ws.addEventListener("open", () => setConnection(true));
  ws.addEventListener("close", () => {
    setConnection(false);
    window.setTimeout(connectWebSocket, 1000);
  });

  ws.addEventListener("message", (event) => {
    try {
      renderStatus(JSON.parse(event.data));
    } catch (err) {
      console.error("invalid ws message", err);
    }
  });
}

async function bootstrap() {
  updateControlVisibility();

  bindSlider(leftSlider, leftValue);
  bindSlider(rightSlider, rightValue);
  bindSlider(linearSlider, linearValue, (v) => Number(v).toFixed(2));
  bindSlider(angularSlider, angularValue, (v) => Number(v).toFixed(2));

  document.querySelectorAll('input[name="driveMode"]').forEach((el) => {
    el.addEventListener("change", () => {
      updateControlVisibility();
      scheduleMotorPush();
    });
  });

  armBtn.addEventListener("click", async () => {
    try {
      if (latestStatus && latestStatus.armed && !latestStatus.estop_asserted) {
        await apiPost("/api/estop");
      } else {
        await apiPost("/api/arm");
      }
    } catch (err) {
      console.error(err);
      alert(`Arm failed: ${err.message}`);
    }
  });

  estopBtn.addEventListener("click", async () => {
    try {
      if (latestStatus && latestStatus.estop_asserted) {
        await apiPost("/api/arm");
      } else {
        await apiPost("/api/estop");
      }
    } catch (err) {
      console.error(err);
      alert(`E-stop failed: ${err.message}`);
    }
  });

  resetEncBtn.addEventListener("click", async () => {
    try {
      await apiPost("/api/reset_encoders");
    } catch (err) {
      console.error(err);
      alert(`Reset failed: ${err.message}`);
    }
  });

  try {
    const res = await fetch("/api/status");
    const status = await res.json();
    renderStatus(status);
  } catch (err) {
    console.error(err);
  }

  connectWebSocket();

  window.setInterval(() => {
    if (latestStatus && latestStatus.armed && !latestStatus.estop_asserted) {
      scheduleMotorPush();
    }
  }, 100);

  window.addEventListener("beforeunload", () => {
    if (ws) {
      ws.close();
    }
    pushZero();
  });
}

bootstrap();
