#!/usr/bin/env python3
import argparse
import socket
import time
from datetime import datetime
from pathlib import Path


def log(name: str, message: str) -> None:
    ts = datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ")
    print(f"[{ts}] [{name}] {message}", flush=True)


def check_tcp(endpoint: str, timeout: float) -> str:
    host, port_str = endpoint.split(":", 1)
    port = int(port_str)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    try:
        sock.connect((host, port))
    except OSError as exc:
        return f"tcp {endpoint} unreachable ({exc})"
    finally:
        sock.close()
    return f"tcp {endpoint} reachable"


def main() -> None:
    parser = argparse.ArgumentParser(description="Runtime stub process for bench bring-up")
    parser.add_argument("--name", required=True)
    parser.add_argument("--interval", type=float, default=2.0)
    parser.add_argument("--check-device", action="append", default=[])
    parser.add_argument("--check-tcp", default="")
    parser.add_argument("--hint", default="")
    parser.add_argument("--timeout", type=float, default=0.35)
    args = parser.parse_args()

    if args.hint:
        log(args.name, args.hint)

    try:
        while True:
            statuses = []

            for dev in args.check_device:
                exists = Path(dev).exists()
                statuses.append(f"{dev}:{'present' if exists else 'missing'}")

            if args.check_tcp:
                statuses.append(check_tcp(args.check_tcp, args.timeout))

            if not statuses:
                statuses.append("alive")

            log(args.name, " | ".join(statuses))
            time.sleep(args.interval)
    except KeyboardInterrupt:
        log(args.name, "stopped")


if __name__ == "__main__":
    main()
