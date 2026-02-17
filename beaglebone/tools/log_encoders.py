#!/usr/bin/env python3
"""Periodic encoder logger for quick bench testing."""

from __future__ import annotations

import argparse
import json
import time
from urllib import request


def fetch_status(base_url: str) -> dict:
    req = request.Request(f"{base_url.rstrip('/')}/api/status", method="GET")
    with request.urlopen(req, timeout=2.0) as resp:
        return json.loads(resp.read().decode("utf-8"))


def main() -> None:
    parser = argparse.ArgumentParser(description="Log encoder counts/velocity from daemon")
    parser.add_argument("--base-url", default="http://127.0.0.1:8080")
    parser.add_argument("--period", type=float, default=0.1)
    args = parser.parse_args()

    print("ts_ms,w0_count,w1_count,w2_count,w3_count,w0_vel,w1_vel,w2_vel,w3_vel")
    while True:
        status = fetch_status(args.base_url)
        enc = status["encoder"]
        counts = enc["counts"]
        vel = enc["velocity_tps"]
        ts = int(time.time() * 1000)
        print(
            f"{ts},{counts[0]},{counts[1]},{counts[2]},{counts[3]},"
            f"{vel[0]},{vel[1]},{vel[2]},{vel[3]}"
        )
        time.sleep(args.period)


if __name__ == "__main__":
    main()
