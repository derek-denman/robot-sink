#!/usr/bin/env python3
"""CLI smoke-test client for bbb_base_daemon.py."""

from __future__ import annotations

import argparse
import json
import time
from urllib import request


def _api(base_url: str, method: str, path: str, payload: dict | None = None) -> dict:
    body = None
    headers = {}
    if payload is not None:
        body = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"

    req = request.Request(
        f"{base_url.rstrip('/')}{path}",
        method=method,
        headers=headers,
        data=body,
    )
    with request.urlopen(req, timeout=2.0) as resp:
        return json.loads(resp.read().decode("utf-8"))


def cmd_status(args: argparse.Namespace) -> None:
    print(json.dumps(_api(args.base_url, "GET", "/api/status"), indent=2))


def cmd_arm(args: argparse.Namespace) -> None:
    print(json.dumps(_api(args.base_url, "POST", "/api/arm", {}), indent=2))


def cmd_disarm(args: argparse.Namespace) -> None:
    print(json.dumps(_api(args.base_url, "POST", "/api/estop", {}), indent=2))


def cmd_motor(args: argparse.Namespace) -> None:
    payload = {"left": args.left, "right": args.right}
    print(json.dumps(_api(args.base_url, "POST", "/api/motor", payload), indent=2))


def cmd_reset(args: argparse.Namespace) -> None:
    print(json.dumps(_api(args.base_url, "POST", "/api/reset_encoders", {}), indent=2))


def cmd_watch(args: argparse.Namespace) -> None:
    while True:
        status = _api(args.base_url, "GET", "/api/status")
        enc = status["encoder"]
        print(
            f"armed={status['armed']} estop={status['estop_asserted']} "
            f"counts={enc['counts']} vel={enc['velocity_tps']}"
        )
        time.sleep(args.period)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="CLI tester for BBB base daemon")
    parser.add_argument("--base-url", default="http://127.0.0.1:8080")

    sub = parser.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("status")
    s.set_defaults(func=cmd_status)

    s = sub.add_parser("arm")
    s.set_defaults(func=cmd_arm)

    s = sub.add_parser("disarm")
    s.set_defaults(func=cmd_disarm)

    s = sub.add_parser("motor")
    s.add_argument("--left", type=int, required=True, help="-1000..1000")
    s.add_argument("--right", type=int, required=True, help="-1000..1000")
    s.set_defaults(func=cmd_motor)

    s = sub.add_parser("reset-encoders")
    s.set_defaults(func=cmd_reset)

    s = sub.add_parser("watch")
    s.add_argument("--period", type=float, default=0.2)
    s.set_defaults(func=cmd_watch)

    return parser


def main() -> None:
    args = build_parser().parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
