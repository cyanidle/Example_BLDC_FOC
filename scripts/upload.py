#!/usr/bin/env python3
"""Flash a firmware image through a running OpenOCD telnet server.

    scripts/upload.py <image> [--host H] [--port P]

ELF images carry their own load addresses; a .bin is written at the flash
base (0x08000000). The server must already be running (scripts/openocd.sh).
Each command runs to completion before the next is sent: OpenOCD prints its
`> ` prompt only when it is ready for more input, and `flash write_image` and
`verify_image` stay silent for seconds while they work (verify is silent
entirely on success), so the prompt is the only reliable end-of-command mark.
"""

import os
import socket
import sys
import time
from argparse import ArgumentParser

HARD_S = 120.0
PROMPT = b"> "

FLASH_BASE = "0x08000000"

FAILURES = (
    "error:",
    "Error:",
    "failed",
    "Failed",
    "Can't open",
    "can't open",
    "couldn't open",
    "contents differ",
    "not halted",
)

parser = ArgumentParser(description=__doc__)
parser.add_argument("file", help="image to flash (ELF or BIN)")
parser.add_argument("--host", default="127.0.0.1")
parser.add_argument("--port", type=int, default=4444)
args = parser.parse_args()


class Ocd:
    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=5.0)

    def read_until_prompt(self, timeout):
        out = b""
        start = time.monotonic()
        while True:
            left = timeout - (time.monotonic() - start)
            if left <= 0:
                return out.decode("utf-8", "replace")
            self.sock.settimeout(left)
            try:
                chunk = self.sock.recv(4096)
            except socket.timeout:
                return out.decode("utf-8", "replace")
            if not chunk:
                raise ConnectionError("OpenOCD closed the connection")
            out += chunk
            if out.endswith(PROMPT):
                return out.decode("utf-8", "replace")

    def command(self, line, timeout=HARD_S):
        self.sock.sendall(line.encode() + b"\n")
        return self.read_until_prompt(timeout)


def main():
    # OpenOCD resolves a relative path against its own working directory, not
    # the caller's, so hand it the absolute path of the image.
    image = os.path.abspath(args.file)
    if image.endswith(".bin"):
        image += " " + FLASH_BASE
    try:
        ocd = Ocd(args.host, args.port)
    except OSError as exc:
        print(f"cannot reach OpenOCD at {args.host}:{args.port}: {exc}", file=sys.stderr)
        return 1
    ocd.read_until_prompt(5.0)
    steps = (
        "reset halt",
        f"flash write_image erase {image}",
        f"verify_image {os.path.abspath(args.file)}",
        "reset run",
    )
    for step in steps:
        print(f"> {step}")
        reply = ocd.command(step)
        sys.stdout.write(reply)
        if not reply.endswith("\n"):
            print()
        if any(marker in reply for marker in FAILURES):
            print(f"upload failed at: {step}", file=sys.stderr)
            return 1
    print(f"flashed {args.file}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
