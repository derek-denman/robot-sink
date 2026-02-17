#!/usr/bin/env python3
"""BeagleBone base daemon: rpmsg bridge + safety logic + REST/WebSocket UI API."""

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

import protocol
from hw import GPIOStopLine, RPMsgEndpoint
from safety import SafetyController


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


class BBBBaseDaemon:
    def __init__(self, config: dict[str, Any], *, dry_run: bool, logger: logging.Logger) -> None:
        self.config = config
        self.dry_run = dry_run
        self.logger = logger

        pru_cfg = config.get("pru", {})
        sab_cfg = config.get("sabertooth", {})
        safety_cfg = config.get("safety", {})
        pins_cfg = config.get("pins", {}).get("sabertooth", {})

        self.encoder_ep = RPMsgEndpoint(
            pru_cfg.get("encoder_rpmsg_dev", "/dev/rpmsg_pru30"),
            dry_run=dry_run,
            logger=logger,
        )
        self.motor_ep = RPMsgEndpoint(
            pru_cfg.get("motor_rpmsg_dev", "/dev/rpmsg_pru31"),
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
        self._sequence = 1
        self._tasks: list[asyncio.Task] = []
        self._ws_clients: set[web.WebSocketResponse] = set()
        self._last_sent_motor: tuple[int, int] | None = None

        self._rx_poll_hz = max(10.0, float(pru_cfg.get("rx_poll_hz", 250)))
        self._encoder_poll_hz = max(1.0, float(pru_cfg.get("encoder_poll_hz", 25)))
        self._motor_tx_hz = max(5.0, float(pru_cfg.get("motor_tx_hz", 40)))
        self._ws_hz = max(2.0, float(config.get("api", {}).get("websocket_hz", 20)))

        self._command_scale = int(config.get("drive", {}).get("command_scale", 1000))
        self._max_linear_mps = float(config.get("drive", {}).get("max_linear_mps", 0.45))
        self._max_angular_radps = float(config.get("drive", {}).get("max_angular_radps", 1.5))
        self._wheel_base_m = float(config.get("drive", {}).get("wheel_base_m", 0.32))

        self._gui_root = Path(__file__).resolve().parents[1] / "gui" / "static"

        self._sab_mode_name = str(sab_cfg.get("mode", "packetized")).lower().strip()
        self._sab_mode = (
            protocol.BBB_SABER_MODE_SIMPLIFIED
            if self._sab_mode_name == "simplified"
            else protocol.BBB_SABER_MODE_PACKETIZED
        )
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

    def _next_seq(self) -> int:
        self._sequence = (self._sequence + 1) & 0xFFFF
        if self._sequence == 0:
            self._sequence = 1
        return self._sequence

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
            },
            "s2_stop_asserted": self.stop_gpio.asserted,
            "dry_run": self.dry_run,
        }

    def _send_cmd(self, endpoint: RPMsgEndpoint, command: int, payload: bytes = b"", *, module: int) -> bool:
        msg = protocol.build_message(
            module=module,
            msg_type=protocol.BBB_MSG_TYPE_COMMAND,
            command=command,
            sequence=self._next_seq(),
            payload=payload,
        )
        return endpoint.send(msg)

    def _send_pru0_params(self) -> None:
        payload = protocol.pack_enc_params(
            self._enc_sample_period_us,
            self._enc_velocity_interval_ms,
            self._enc_stream_hz,
        )
        self._send_cmd(self.encoder_ep, protocol.BBB_CMD_SET_ENC_PARAMS, payload, module=protocol.BBB_MODULE_HOST)
        self._send_cmd(
            self.encoder_ep,
            protocol.BBB_CMD_SET_STREAM,
            protocol.pack_stream_config(self._enc_stream_hz),
            module=protocol.BBB_MODULE_HOST,
        )

    def _send_pru1_params(self) -> None:
        payload = protocol.pack_sabertooth_params(
            baud=self._sab_baud,
            mode=self._sab_mode,
            address=self._sab_address,
            tx_r30_bit=self._sab_tx_r30_bit,
            watchdog_ms=self._sab_watchdog_ms,
            safe_stop_hold_ms=int(self.config.get("safety", {}).get("safe_stop_hold_ms", 300)),
            ramp_step=self._sab_ramp_step,
            control_period_us=self._sab_control_period_us,
        )
        self._send_cmd(self.motor_ep, protocol.BBB_CMD_SET_SABER_PARAMS, payload, module=protocol.BBB_MODULE_HOST)

    def _send_pru1_estop(self, asserted: bool) -> None:
        self._send_cmd(
            self.motor_ep,
            protocol.BBB_CMD_SET_ESTOP,
            protocol.pack_estop(asserted),
            module=protocol.BBB_MODULE_HOST,
        )

    def _send_pru1_motor(self, left: int, right: int) -> None:
        self._send_cmd(
            self.motor_ep,
            protocol.BBB_CMD_SET_MOTOR,
            protocol.pack_motor(left, right),
            module=protocol.BBB_MODULE_HOST,
        )

    async def start(self) -> None:
        self.encoder_ep.open()
        self.motor_ep.open()

        self.stop_gpio.setup()
        self.stop_gpio.assert_stop()

        # Safe by default: force estop state on startup.
        self._send_pru1_params()
        self._send_pru0_params()
        self._send_pru1_estop(True)
        self._send_pru1_motor(0, 0)

        self._tasks = [
            asyncio.create_task(self._rx_loop(), name="rx_loop"),
            asyncio.create_task(self._encoder_poll_loop(), name="encoder_poll_loop"),
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
        self.encoder_ep.close()
        self.motor_ep.close()
        self._log_event("daemon_stopped")

    async def _trip_estop(self, reason: str) -> None:
        async with self._lock:
            now = time.monotonic()
            self.safety.trip_estop(now, reason)
            self.state.left_cmd = 0
            self.state.right_cmd = 0

            self.stop_gpio.assert_stop()
            self._send_pru1_estop(True)
            self._send_pru1_motor(0, 0)
            self._log_event("estop_asserted", reason=reason)

    async def _arm(self) -> tuple[bool, str]:
        async with self._lock:
            now = time.monotonic()
            if not self.safety.arm(now):
                return False, "safe_stop_hold"

            self.state.left_cmd = 0
            self.state.right_cmd = 0
            self._send_pru1_estop(False)
            self._send_pru1_motor(0, 0)
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

    async def _rx_loop(self) -> None:
        period = 1.0 / self._rx_poll_hz
        while True:
            self._drain_endpoint(self.encoder_ep, protocol.BBB_MODULE_PRU0)
            self._drain_endpoint(self.motor_ep, protocol.BBB_MODULE_PRU1)
            await asyncio.sleep(period)

    def _drain_endpoint(self, endpoint: RPMsgEndpoint, expected_module: int) -> None:
        for raw in endpoint.recv_all():
            msg = protocol.parse_message(raw)
            if msg is None:
                continue
            if msg.module != expected_module:
                continue

            if expected_module == protocol.BBB_MODULE_PRU0:
                self.state.pru0_online = True
                self._handle_pru0_message(msg)
            else:
                self.state.pru1_online = True
                self._handle_pru1_message(msg)

    def _handle_pru0_message(self, msg: protocol.Message) -> None:
        if msg.command != protocol.BBB_CMD_GET_SNAPSHOT:
            return

        snapshot = protocol.unpack_encoder_snapshot(msg.payload)
        if snapshot is None:
            return

        self.state.encoder_counts = snapshot["counts"]
        self.state.encoder_velocity_tps = snapshot["velocity_tps"]
        self.state.encoder_timestamp_us = snapshot["timestamp_us"]
        self.state.encoder_updated_monotonic = time.monotonic()

    def _handle_pru1_message(self, msg: protocol.Message) -> None:
        if msg.command not in (protocol.BBB_CMD_ACK, protocol.BBB_CMD_NACK):
            return

        ack = protocol.unpack_ack(msg.payload)
        if ack is None:
            return

        if msg.command == protocol.BBB_CMD_NACK:
            self._log_event(
                "pru1_nack",
                sequence=msg.sequence,
                detail=ack["detail"],
                status=ack["status"],
            )

    async def _encoder_poll_loop(self) -> None:
        period = 1.0 / self._encoder_poll_hz
        while True:
            self._send_cmd(
                self.encoder_ep,
                protocol.BBB_CMD_GET_SNAPSHOT,
                b"",
                module=protocol.BBB_MODULE_HOST,
            )
            await asyncio.sleep(period)

    async def _motor_tx_loop(self) -> None:
        period = 1.0 / self._motor_tx_hz
        while True:
            safety = self.safety.snapshot()
            if safety.armed and not safety.estop_asserted:
                left = self.state.left_cmd
                right = self.state.right_cmd
            else:
                left = 0
                right = 0

            if (safety.armed and not safety.estop_asserted) or self._last_sent_motor != (left, right):
                self._send_pru1_motor(left, right)
                self._last_sent_motor = (left, right)
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
        self._send_cmd(
            self.encoder_ep,
            protocol.BBB_CMD_RESET_COUNTS,
            b"",
            module=protocol.BBB_MODULE_HOST,
        )
        return web.json_response({"ok": True})

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
