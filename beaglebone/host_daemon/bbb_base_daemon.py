#!/usr/bin/env python3
"""BeagleBone base daemon: UIO bridge + safety logic + REST/WebSocket API."""

from __future__ import annotations

import argparse
import asyncio
from dataclasses import dataclass, field
import json
import logging
from pathlib import Path
import time
from typing import Any

from aiohttp import web, WSMsgType
import yaml

from hw import GPIOStopLine
from safety import SafetyController
from uio_transport import UIOTransport


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def build_logger(level: str) -> logging.Logger:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(message)s",
    )
    return logging.getLogger("bbb_base_daemon")


@dataclass
class DaemonState:
    left_cmd: int = 0
    right_cmd: int = 0
    encoder_counts: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    encoder_velocity_tps: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    encoder_timestamp_us: int = 0
    encoder_updated_monotonic: float = 0.0
    last_motor_update_monotonic: float = 0.0
    pru0_online: bool = False
    pru1_online: bool = False
    motor_applied_left: int = 0
    motor_applied_right: int = 0


class BBBBaseDaemon:
    def __init__(self, config: dict[str, Any], *, dry_run: bool, logger: logging.Logger) -> None:
        self.config = config
        self.dry_run = dry_run
        self.logger = logger

        pru_cfg = config.get("pru", {})
        uio_cfg = pru_cfg.get("uio", {})
        sab_cfg = config.get("sabertooth", {})
        safety_cfg = config.get("safety", {})
        pins_cfg = config.get("pins", {}).get("sabertooth", {})

        self.transport = UIOTransport(
            uio_cfg,
            dry_run=dry_run,
            logger=logger,
        )

        self.stop_gpio = GPIOStopLine(
            int(pins_cfg.get("s2_stop_gpio_number", 48)),
            active_high=bool(pins_cfg.get("s2_stop_active_high", True)),
            dry_run=dry_run,
            logger=logger,
        )

        self.safety = SafetyController(
            command_timeout_ms=int(safety_cfg.get("command_timeout_ms", 200)),
            safe_stop_hold_ms=int(safety_cfg.get("safe_stop_hold_ms", 300)),
        )

        self.state = DaemonState()
        self._lock = asyncio.Lock()
        self._tasks: list[asyncio.Task] = []
        self._ws_clients: set[web.WebSocketResponse] = set()
        self._last_sent_motor: tuple[int, int, bool, bool] | None = None

        self._rx_poll_hz = max(10.0, float(pru_cfg.get("rx_poll_hz", 250)))
        self._motor_tx_hz = max(5.0, float(pru_cfg.get("motor_tx_hz", 40)))
        self._ws_hz = max(2.0, float(config.get("api", {}).get("websocket_hz", 20)))

        self._command_scale = int(config.get("drive", {}).get("command_scale", 1000))
        self._max_linear_mps = float(config.get("drive", {}).get("max_linear_mps", 0.45))
        self._max_angular_radps = float(config.get("drive", {}).get("max_angular_radps", 1.5))
        self._wheel_base_m = float(config.get("drive", {}).get("wheel_base_m", 0.32))

        self._gui_root = Path(__file__).resolve().parents[1] / "gui" / "static"

        self._sab_mode_name = str(sab_cfg.get("mode", "packetized")).lower().strip()
        self._sab_mode = 0 if self._sab_mode_name == "simplified" else 1
        self._sab_baud = int(sab_cfg.get("baud", 38400))
        self._sab_address = int(sab_cfg.get("address", 128))
        self._sab_tx_r30_bit = int(sab_cfg.get("tx_pru1_r30_bit", 0))
        self._sab_ramp_step = int(sab_cfg.get("ramp_step", 40))
        self._sab_watchdog_ms = int(sab_cfg.get("pru_watchdog_ms", 200))
        self._sab_control_period_us = int(sab_cfg.get("control_period_us", 10000))

        self._enc_velocity_interval_ms = int(config.get("encoder", {}).get("velocity_interval_ms", 20))
        self._enc_sample_period_us = int(config.get("encoder", {}).get("sample_period_us", 20))
        self._enc_stream_hz = int(config.get("encoder", {}).get("stream_hz", 20))

    def _log_event(self, event: str, **fields: Any) -> None:
        payload = {"event": event, **fields}
        self.logger.info(json.dumps(payload, sort_keys=True))

    def _status(self) -> dict[str, Any]:
        now = time.monotonic()
        safety = self.safety.snapshot()
        return {
            "armed": safety.armed,
            "estop_asserted": safety.estop_asserted,
            "watchdog_tripped": safety.watchdog_tripped,
            "estop_reason": safety.estop_reason,
            "left_cmd": self.state.left_cmd,
            "right_cmd": self.state.right_cmd,
            "encoder": {
                "counts": self.state.encoder_counts,
                "velocity_tps": self.state.encoder_velocity_tps,
                "timestamp_us": self.state.encoder_timestamp_us,
                "age_ms": round((now - self.state.encoder_updated_monotonic) * 1000.0, 1)
                if self.state.encoder_updated_monotonic
                else None,
            },
            "last_motor_command_age_ms": round(
                (now - self.safety.last_command_time) * 1000.0,
                1,
            ),
            "pru": {
                "encoder_online": self.state.pru0_online,
                "motor_online": self.state.pru1_online,
                "transport_error": self.transport.last_error or None,
            },
            "motor_feedback": {
                "applied_left": self.state.motor_applied_left,
                "applied_right": self.state.motor_applied_right,
            },
            "s2_stop_asserted": self.stop_gpio.asserted,
            "dry_run": self.dry_run,
        }

    def _configure_transport(self) -> None:
        self.transport.configure_encoder(
            sample_period_us=self._enc_sample_period_us,
            velocity_interval_ms=self._enc_velocity_interval_ms,
            stream_hz=self._enc_stream_hz,
        )
        self.transport.configure_motor(
            baud=self._sab_baud,
            mode=self._sab_mode,
            address=self._sab_address,
            tx_r30_bit=self._sab_tx_r30_bit,
            watchdog_ms=self._sab_watchdog_ms,
            safe_stop_hold_ms=int(self.config.get("safety", {}).get("safe_stop_hold_ms", 300)),
            ramp_step=self._sab_ramp_step,
            control_period_us=self._sab_control_period_us,
        )
        self.transport.write_motor_command(0, 0, armed=False, estop=True)

    async def start(self) -> None:
        self.transport.open()

        self.stop_gpio.setup()
        self.stop_gpio.assert_stop()
        self._configure_transport()

        self._tasks = [
            asyncio.create_task(self._telemetry_loop(), name="telemetry_loop"),
            asyncio.create_task(self._motor_tx_loop(), name="motor_tx_loop"),
            asyncio.create_task(self._watchdog_loop(), name="watchdog_loop"),
            asyncio.create_task(self._ws_broadcast_loop(), name="ws_broadcast_loop"),
        ]
        self._log_event("daemon_started", dry_run=self.dry_run)

    async def stop(self) -> None:
        for task in self._tasks:
            task.cancel()
        for task in self._tasks:
            try:
                await task
            except asyncio.CancelledError:
                pass

        self.stop_gpio.assert_stop()
        self.transport.write_motor_command(0, 0, armed=False, estop=True)
        self.transport.close()
        self._log_event("daemon_stopped")

    async def _trip_estop(self, reason: str) -> None:
        async with self._lock:
            now = time.monotonic()
            self.safety.trip_estop(now, reason)
            self.state.left_cmd = 0
            self.state.right_cmd = 0

            self.stop_gpio.assert_stop()
            self.transport.write_motor_command(0, 0, armed=False, estop=True)
            self._log_event("estop_asserted", reason=reason)

    async def _arm(self) -> tuple[bool, str]:
        async with self._lock:
            now = time.monotonic()
            if not self.safety.arm(now):
                return False, "safe_stop_hold"

            self.state.left_cmd = 0
            self.state.right_cmd = 0
            self.transport.write_motor_command(0, 0, armed=True, estop=False)
            self.stop_gpio.release_stop()
            self._log_event("armed")
            return True, ""

    def _decode_left_right(self, payload: dict[str, Any]) -> tuple[int, int]:
        if "left" in payload and "right" in payload:
            left_f = float(payload["left"])
            right_f = float(payload["right"])

            if abs(left_f) <= 1.0 and abs(right_f) <= 1.0:
                left = int(clamp(left_f, -1.0, 1.0) * self._command_scale)
                right = int(clamp(right_f, -1.0, 1.0) * self._command_scale)
            else:
                left = int(clamp(left_f, -self._command_scale, self._command_scale))
                right = int(clamp(right_f, -self._command_scale, self._command_scale))
            return left, right

        linear = float(payload.get("linear", 0.0))
        angular = float(payload.get("angular", 0.0))

        angular = clamp(angular, -self._max_angular_radps, self._max_angular_radps)
        linear = clamp(linear, -self._max_linear_mps, self._max_linear_mps)

        half_track = 0.5 * self._wheel_base_m
        left_mps = linear - angular * half_track
        right_mps = linear + angular * half_track

        left_norm = clamp(left_mps / self._max_linear_mps, -1.0, 1.0)
        right_norm = clamp(right_mps / self._max_linear_mps, -1.0, 1.0)

        return int(left_norm * self._command_scale), int(right_norm * self._command_scale)

    async def _telemetry_loop(self) -> None:
        period = 1.0 / self._rx_poll_hz
        while True:
            snapshot = self.transport.read_encoder_snapshot()
            if snapshot is not None:
                timestamp_us = int(snapshot.get("timestamp_us", 0))
                if timestamp_us != self.state.encoder_timestamp_us:
                    self.state.encoder_updated_monotonic = time.monotonic()

                self.state.encoder_timestamp_us = timestamp_us
                self.state.encoder_counts = list(snapshot.get("counts", [0, 0, 0, 0]))
                self.state.encoder_velocity_tps = list(snapshot.get("velocity_tps", [0, 0, 0, 0]))
                self.state.pru0_online = bool(snapshot.get("pru0_online", False))
                self.state.pru1_online = bool(snapshot.get("pru1_online", False))
                self.state.motor_applied_left = int(snapshot.get("motor_applied_left", 0))
                self.state.motor_applied_right = int(snapshot.get("motor_applied_right", 0))
            else:
                self.state.pru0_online = False
                self.state.pru1_online = False

            await asyncio.sleep(period)

    async def _motor_tx_loop(self) -> None:
        period = 1.0 / self._motor_tx_hz
        while True:
            safety = self.safety.snapshot()
            armed = safety.armed and not safety.estop_asserted
            estop = not armed

            if armed:
                left = self.state.left_cmd
                right = self.state.right_cmd
            else:
                left = 0
                right = 0

            command_key = (left, right, armed, estop)
            if armed or self._last_sent_motor != command_key:
                self.transport.write_motor_command(left, right, armed=armed, estop=estop)
                self._last_sent_motor = command_key

            await asyncio.sleep(period)

    async def _watchdog_loop(self) -> None:
        while True:
            now = time.monotonic()
            if self.safety.check_command_timeout(now):
                await self._trip_estop("command_timeout")
            await asyncio.sleep(0.02)

    async def _ws_broadcast_loop(self) -> None:
        period = 1.0 / self._ws_hz
        while True:
            if self._ws_clients:
                payload = self._status()
                stale: list[web.WebSocketResponse] = []
                for ws in list(self._ws_clients):
                    try:
                        await ws.send_json(payload)
                    except Exception:
                        stale.append(ws)
                for ws in stale:
                    self._ws_clients.discard(ws)
            await asyncio.sleep(period)

    async def api_status(self, _: web.Request) -> web.Response:
        return web.json_response(self._status())

    async def api_arm(self, _: web.Request) -> web.Response:
        ok, reason = await self._arm()
        if not ok:
            return web.json_response(
                {"ok": False, "reason": reason, "status": self._status()},
                status=409,
            )
        return web.json_response({"ok": True, "status": self._status()})

    async def api_estop(self, _: web.Request) -> web.Response:
        await self._trip_estop("api_estop")
        return web.json_response({"ok": True, "status": self._status()})

    async def api_motor(self, request: web.Request) -> web.Response:
        try:
            payload = await request.json()
        except Exception:
            payload = {}

        async with self._lock:
            safety = self.safety.snapshot()
            if not safety.armed or safety.estop_asserted:
                return web.json_response(
                    {"ok": False, "error": "not_armed", "status": self._status()},
                    status=409,
                )

            try:
                left, right = self._decode_left_right(payload)
            except Exception:
                return web.json_response(
                    {"ok": False, "error": "bad_motor_payload"},
                    status=400,
                )
            self.state.left_cmd = left
            self.state.right_cmd = right
            now = time.monotonic()
            self.safety.register_command(now)
            self.state.last_motor_update_monotonic = now

        return web.json_response({"ok": True, "status": self._status()})

    async def api_reset_encoders(self, _: web.Request) -> web.Response:
        ok = self.transport.reset_encoders()
        return web.json_response({"ok": bool(ok)})

    async def ws_handler(self, request: web.Request) -> web.StreamResponse:
        ws = web.WebSocketResponse(heartbeat=20)
        await ws.prepare(request)
        self._ws_clients.add(ws)
        await ws.send_json(self._status())

        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT and msg.data == "ping":
                    await ws.send_str("pong")
                elif msg.type == WSMsgType.ERROR:
                    break
        finally:
            self._ws_clients.discard(ws)

        return ws

    async def index_handler(self, _: web.Request) -> web.Response:
        return web.FileResponse(self._gui_root / "index.html")


def load_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def build_app(daemon: BBBBaseDaemon) -> web.Application:
    app = web.Application()

    app.router.add_get("/api/status", daemon.api_status)
    app.router.add_post("/api/arm", daemon.api_arm)
    app.router.add_post("/api/estop", daemon.api_estop)
    app.router.add_post("/api/motor", daemon.api_motor)
    app.router.add_post("/api/reset_encoders", daemon.api_reset_encoders)
    app.router.add_get("/ws", daemon.ws_handler)

    app.router.add_get("/", daemon.index_handler)
    app.router.add_static("/static/", str(daemon._gui_root), show_index=False)

    app.on_startup.append(lambda _: daemon.start())
    app.on_cleanup.append(lambda _: daemon.stop())
    return app


def parse_args() -> argparse.Namespace:
    default_cfg = Path(__file__).resolve().with_name("config.yaml")

    parser = argparse.ArgumentParser(description="BBB base daemon")
    parser.add_argument("--config", type=Path, default=default_cfg)
    parser.add_argument("--host", type=str, default=None)
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--log-level", type=str, default="INFO")
    parser.add_argument("--dry-run", action="store_true", help="Run full stack without touching hardware")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logger = build_logger(args.log_level)

    config = load_config(args.config)
    daemon = BBBBaseDaemon(config, dry_run=args.dry_run, logger=logger)
    app = build_app(daemon)

    host = args.host if args.host else str(config.get("api", {}).get("host", "0.0.0.0"))
    port = args.port if args.port else int(config.get("api", {}).get("port", 8080))

    web.run_app(app, host=host, port=port)


if __name__ == "__main__":
    main()
