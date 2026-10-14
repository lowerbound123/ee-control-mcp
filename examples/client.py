"""Official MCP HTTP client; observations by default, optional original motion demo."""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
import os
from pathlib import Path

import httpx2
from mcp import ClientSession
from mcp.client.streamable_http import streamable_http_client


async def run(args) -> None:
    token = os.environ.get("EE_CONTROL_MCP_TOKEN")
    headers = {"Authorization": f"Bearer {token}"} if token else {}
    async with httpx2.AsyncClient(headers=headers, trust_env=False, timeout=120) as http:
        async with streamable_http_client(args.url, http_client=http) as streams:
            async with ClientSession(streams[0], streams[1]) as session:
                await session.initialize()
                tools = await session.list_tools()
                print("Tools:", ", ".join(tool.name for tool in tools.tools))

                async def call(name, arguments):
                    result = await session.call_tool(name, arguments)
                    data = result.structured_content
                    if data is None:
                        data = json.loads(
                            next(block.text for block in result.content if block.type == "text")
                        )
                    print(name, json.dumps(data, ensure_ascii=False, indent=2))
                    if result.is_error:
                        raise RuntimeError(data.get("error", "MCP tool failed"))
                    return data, result

                await call("props", {})
                if args.execute_demo:
                    data, _ = await call(
                        "move_eef",
                        {
                            "targets": {
                                "left": {"end_pos": [0.1, 0.03, 0.05, 0, 0, 0], "gripper": 1},
                                "right": {"end_pos": [0.05, 0, 0.1, 0, 0, 0], "gripper": 1},
                            },
                            "wait": True,
                            "timeout_s": 120,
                        },
                    )
                    if data["status"] != "completed":
                        raise RuntimeError("EEF demo is still running; home was not submitted")
                    await asyncio.sleep(3)
                    data, _ = await call(
                        "send_command",
                        {
                            "commands": "both:home",
                            "wait": True,
                            "timeout_s": 120,
                        },
                    )
                    if data["status"] != "completed":
                        raise RuntimeError("Home is still running; inspect state for progress")

                data, result = await call(
                    "state",
                    {
                        "include_images": args.images or args.image_dir is not None,
                        "cameras": args.cameras,
                    },
                )
                image_blocks = [block for block in result.content if block.type == "image"]
                for metadata, block in zip(data.get("images", []), image_blocks, strict=True):
                    print(f"JPEG {metadata['camera']}: {metadata['width']}x{metadata['height']}")
                    if args.image_dir is not None:
                        args.image_dir.mkdir(parents=True, exist_ok=True)
                        path = args.image_dir / f"{metadata['camera']}.jpg"
                        path.write_bytes(base64.b64decode(block.data))
                        print("Saved:", path.resolve())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:3000/mcp")
    parser.add_argument("--images", action="store_true", help="request JPEG image blocks")
    parser.add_argument("--image-dir", type=Path, help="optionally save the requested images")
    parser.add_argument(
        "--cameras", nargs="+", choices=("head", "left_wrist", "right_wrist"), default=["head"]
    )
    parser.add_argument(
        "--execute-demo",
        action="store_true",
        help="execute dual EEF targets, wait 3 s, then both:home",
    )
    asyncio.run(run(parser.parse_args()))


if __name__ == "__main__":
    main()
