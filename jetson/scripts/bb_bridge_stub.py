#!/usr/bin/env python3
import argparse
import json
import socket
import time
from datetime import datetime


def now() -> str:
    return datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ")


def main() -> None:
    parser = argparse.ArgumentParser(description="Simple BeagleBone UDP heartbeat stub")
    parser.add_argument("--host", default="192.168.7.2", help="BeagleBone host")
    parser.add_argument("--port", type=int, default=8765, help="BeagleBone UDP port")
    parser.add_argument("--interval", type=float, default=1.0, help="heartbeat period seconds")
    parser.add_argument("--timeout", type=float, default=0.25, help="recv timeout seconds")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(args.timeout)

    print(
        f"[{now()}] [bb-bridge] sending heartbeat to {args.host}:{args.port}",
        flush=True,
    )

    seq = 0
    try:
        while True:
            payload = {
                "type": "jetson_heartbeat",
                "seq": seq,
                "timestamp": now(),
                "transport": "udp",
            }
            message = json.dumps(payload).encode("utf-8")
            sock.sendto(message, (args.host, args.port))

            ack = "no-ack"
            try:
                data, _ = sock.recvfrom(512)
                ack = f"ack:{data.decode('utf-8', errors='replace').strip()}"
            except socket.timeout:
                pass

            print(
                f"[{now()}] [bb-bridge] seq={seq} tx={len(message)}B {ack}",
                flush=True,
            )

            seq += 1
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print(f"[{now()}] [bb-bridge] stopped", flush=True)


if __name__ == "__main__":
    main()
