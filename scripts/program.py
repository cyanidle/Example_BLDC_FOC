#!/usr/bin/env python3

import asyncio, telnetlib3
import sys
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument("file")
parser.add_argument("--host", default="127.0.0.1")
parser.add_argument("--port", default=4444)
args = parser.parse_args()

async def main():
    reader, writer = await telnetlib3.open_connection(host=args.host, port=args.port)
    writer.write("reset halt\n")
    await asyncio.sleep(0.5)
    writer.write(f"flash write_image erase {args.file}\n")
    resp = await asyncio.wait_for(reader.readuntil(b'wrote'), timeout=50.0)
    writer.write("reset halt\n")
    await asyncio.sleep(0.5)
    print(resp.decode(), file=sys.stderr)

asyncio.run(main())
