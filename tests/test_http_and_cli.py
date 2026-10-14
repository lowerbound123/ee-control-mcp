import asyncio
import base64
import csv
import io
import json
import os
import re
import socket
import subprocess
import sys
import threading
import time
from contextlib import asynccontextmanager, contextmanager
from dataclasses import replace
from pathlib import Path

import httpx
import httpx2
import pytest
import uvicorn
from mcp import ClientSession
from mcp.client.streamable_http import streamable_http_client
from PIL import Image

from ee_control_mcp.config import Config, load_config


@pytest.fixture(autouse=True)
def observation_working_directory(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)


@contextmanager
def running_http(*, readonly=False, token=None, backend="sim", config=None):
    from ee_control_mcp.runtime import Runtime
    from ee_control_mcp.server import create_app

    config = Config() if config is None else config
    config = replace(config, control=replace(config.control, publish_rate_hz=1000))
    runtime = Runtime(config, backend=backend, actuation_enabled=not readonly)
    app = create_app(runtime, token=token)
    listener = socket.socket()
    listener.bind(("127.0.0.1", 0))
    port = listener.getsockname()[1]
    server = uvicorn.Server(
        uvicorn.Config(app, log_level="error", timeout_graceful_shutdown=2, ws="none")
    )
    thread = threading.Thread(target=server.run, kwargs={"sockets": [listener]}, daemon=True)
    thread.start()
    deadline = time.monotonic() + 5
    try:
        while not server.started:
            if not thread.is_alive():
                pytest.fail("HTTP server failed to start")
            if time.monotonic() >= deadline:
                pytest.fail("HTTP startup timed out")
            time.sleep(0.01)
        yield f"http://127.0.0.1:{port}", runtime
    finally:
        server.should_exit = True
        thread.join(5)
        listener.close()
        assert not thread.is_alive()


@asynccontextmanager
async def session(url, token=None, headers=None):
    headers = dict(headers or {})
    if token:
        headers["Authorization"] = f"Bearer {token}"
    async with httpx2.AsyncClient(headers=headers, trust_env=False) as http:
        async with streamable_http_client(url + "/mcp", http_client=http) as streams:
            async with ClientSession(streams[0], streams[1]) as client:
                await client.initialize()
                yield client


def payload(result):
    return result.structured_content or json.loads(result.content[0].text)


def read_state_rows(directory):
    with (directory / "state.csv").open(newline="", encoding="utf-8") as source:
        return list(csv.DictReader(source))


def test_http_observations_append_state_and_deduplicate_each_camera(tmp_path, monkeypatch):
    from test_observations_and_backends import fake_ros, put_feedback

    fake_ros(monkeypatch)
    config = Config()
    config = replace(
        config,
        observation=replace(config.observation, feedback_max_age_s=60, image_max_age_s=60),
    )
    buffer = io.BytesIO()
    Image.new("RGB", (16, 12), (255, 0, 0)).save(buffer, "PNG")
    image_data = buffer.getvalue()

    with running_http(backend="ros", readonly=True, config=config) as (url, runtime):
        put_feedback(runtime.cache, "left")
        put_feedback(runtime.cache, "right", gripper=3.5)
        for camera in ("head", "left_wrist", "right_wrist"):
            runtime.cache.update_camera(camera, image_data, stamp=77.5)

        async def scenario():
            async with session(url) as client:
                await client.call_tool("props", {})
                with httpx.Client(trust_env=False) as http:
                    assert http.get(url + "/health").status_code == 200
                assert not list(tmp_path.glob("logs/system-*"))

                response = await client.call_tool("state", {"include_images": True})
                first_state = payload(response)
                directories = list(tmp_path.glob("logs/system-*"))
                assert len(directories) == 1, "MCP observations must create a run log directory"
                directory = directories[0]
                assert re.fullmatch(r"system-\d{8}-\d{6}-\d{6}", directory.name)
                rows = read_state_rows(directory)
                assert len(rows) == 1
                assert json.loads(rows[0]["state_json"]) == first_state
                assert float(rows[0]["timestamp"]) == first_state["timestamp"]
                assert float(rows[0]["qpos_6"]) == 2.5
                assert float(rows[0]["qpos_13"]) == 3.5
                assert float(rows[0]["eef_0"]) == 0.1
                assert float(rows[0]["qvel_0"]) == 7
                assert float(rows[0]["effort_13"]) == 20
                assert len(list(directory.glob("*.jpg"))) == 3
                aliases = {"head": "head", "left_wrist": "left", "right_wrist": "right"}
                images = [block for block in response.content if block.type == "image"]
                for metadata, block in zip(first_state["images"], images, strict=True):
                    saved = list(directory.glob(f"{aliases[metadata['camera']]}-*.jpg"))
                    assert len(saved) == 1
                    assert re.fullmatch(r"(left|head|right)-\d{8}-\d{6}-\d{6}\.jpg", saved[0].name)
                    assert saved[0].read_bytes() == base64.b64decode(block.data)

                repeats = await asyncio.gather(
                    *(
                        client.call_tool(
                            "state",
                            {
                                "include_images": True,
                                "image_quality": 60,
                                "cameras": ["right_wrist", "head", "left_wrist"],
                            },
                        )
                        for _ in range(8)
                    )
                )
                assert all(not response.is_error for response in repeats)
                assert len(read_state_rows(directory)) == 1
                assert len(list(directory.glob("*.jpg"))) == 3

                # New feedback is logged even when its pose and ROS stamp are unchanged.
                put_feedback(runtime.cache, "left")
                updated = payload(await client.call_tool("state", {"include_images": True}))
                rows = read_state_rows(directory)
                assert [int(row["seq"]) for row in rows] == [first_state["seq"], updated["seq"]]
                assert len(list(directory.glob("*.jpg"))) == 3

                # Camera receipt sequences are independent of feedback and encoding quality.
                runtime.cache.update_camera("head", image_data, stamp=77.5)
                await client.call_tool("state", {"include_images": True, "cameras": ["head"]})
                assert len(list(directory.glob("head-*.jpg"))) == 2
                assert len(read_state_rows(directory)) == 2
                runtime.cache.update_camera("left_wrist", image_data)
                await client.call_tool("state", {})
                await client.call_tool("state", {"include_images": True, "cameras": ["head"]})
                assert len(list(directory.glob("left-*.jpg"))) == 1
                await client.call_tool("state", {"include_images": True, "cameras": ["left_wrist"]})
                assert len(list(directory.glob("left-*.jpg"))) == 2

        asyncio.run(scenario())


def test_http_logs_missing_feedback_and_skips_unusable_images(tmp_path, monkeypatch):
    from test_observations_and_backends import fake_ros

    fake_ros(monkeypatch)
    with running_http(backend="ros", readonly=True) as (url, runtime):
        runtime.cache.update_camera("head", b"corrupt JPEG")
        runtime.cache.update_camera("left_wrist", b"old JPEG", received_at=time.monotonic() - 120)

        async def scenario():
            async with session(url) as client:
                response = await client.call_tool("state", {"include_images": True})
                state = payload(response)
                assert not response.is_error
                assert state["qpos"] is None
                assert state["cameras_missing"] == ["right_wrist"]
                assert state["cameras_stale"] == ["left_wrist"]
                assert "head" in state["camera_errors"]
                directories = list(tmp_path.glob("logs/system-*"))
                assert len(directories) == 1, "Missing feedback must still be logged as state"
                rows = read_state_rows(directories[0])
                assert len(rows) == 1 and rows[0]["qpos_0"] == ""
                assert json.loads(rows[0]["state_json"]) == state
                assert not list(directories[0].glob("*.jpg"))
                await client.call_tool("state", {"include_images": True})
                assert len(read_state_rows(directories[0])) == 1

        asyncio.run(scenario())


def test_http_observation_log_failure_keeps_state_and_retries_same_sequences(tmp_path, monkeypatch):
    from test_observations_and_backends import fake_ros, put_feedback

    fake_ros(monkeypatch)
    blocked = tmp_path / "logs"
    blocked.write_text("not a directory", encoding="utf-8")
    with running_http(backend="ros", readonly=True) as (url, runtime):
        put_feedback(runtime.cache, "left")
        put_feedback(runtime.cache, "right")
        buffer = io.BytesIO()
        Image.new("RGB", (16, 12), "red").save(buffer, "JPEG")
        runtime.cache.update_camera("head", buffer.getvalue())

        async def scenario():
            async with session(url) as client:
                response = await client.call_tool("state", {"include_images": True})
                assert not response.is_error
                assert payload(response)["state_ready"]
                assert payload(response).get("observation_log_errors"), "Expose disk write failures"
                blocked.rename(tmp_path / "blocked-logs")
                retry = await client.call_tool("state", {"include_images": True})
                assert not retry.is_error and "observation_log_errors" not in payload(retry)
                assert payload(retry)["seq"] == payload(response)["seq"]
                directories = list(tmp_path.glob("logs/system-*"))
                assert len(directories) == 1
                assert len(read_state_rows(directories[0])) == 1
                assert len(list(directories[0].glob("head-*.jpg"))) == 1

        asyncio.run(scenario())


@pytest.mark.parametrize(
    "tool,arguments",
    [
        ("send_command", {"commands": "both:home"}),
        ("move_eef", {"targets": {"left": {"end_pos": [0.2, 0.1, 0.201, 0, 0, 0]}}}),
        ("move_joints", {"targets": {"left": {"joint_pos": [0.001, 0, 0, 0, 0, 0]}}}),
    ],
)
def test_http_control_results_log_returned_state_and_images(tmp_path, tool, arguments):
    with running_http() as (url, runtime):

        async def scenario():
            async with session(url) as client:
                response = await client.call_tool(
                    tool, {**arguments, "wait": True, "include_images": True, "cameras": ["head"]}
                )
                assert not response.is_error and payload(response)["status"] == "completed"
                directories = list(tmp_path.glob("logs/system-*"))
                assert len(directories) == 1, "Control results must log their observations too"
                rows = read_state_rows(directories[0])
                assert len(rows) == 1
                assert json.loads(rows[0]["state_json"]) == payload(response)
                images = [block for block in response.content if block.type == "image"]
                saved = list(directories[0].glob("head-*.jpg"))
                assert len(saved) == len(images) == 1
                assert saved[0].read_bytes() == base64.b64decode(images[0].data)

        asyncio.run(scenario())


def test_http_control_success_survives_observation_log_failure(tmp_path):
    (tmp_path / "logs").write_text("not a directory", encoding="utf-8")
    with running_http() as (url, runtime):

        async def scenario():
            async with session(url) as client:
                response = await client.call_tool(
                    "send_command", {"commands": "both:home", "wait": True}
                )
                assert not response.is_error
                state = payload(response)
                assert state["accepted"] and state["status"] == "completed"
                assert state.get("observation_log_errors"), "Keep the accepted control result"

        asyncio.run(scenario())


def test_http_restarts_create_separate_run_logs(tmp_path):
    for _ in range(2):
        with running_http(readonly=True) as (url, runtime):

            async def scenario():
                async with session(url) as client:
                    await client.call_tool("state", {})

            asyncio.run(scenario())
    directories = list(tmp_path.glob("logs/system-*"))
    assert len(directories) == 2, "Separate server runs must not share or overwrite logs"
    assert all(len(read_state_rows(directory)) == 1 for directory in directories)


def test_real_http_mcp_handshake_tools_control_and_camera_images():
    async def scenario(url):
        async with session(url) as client:
            listing = await client.list_tools()
            assert {tool.name for tool in listing.tools} == {
                "props",
                "state",
                "send_command",
                "move_eef",
                "move_joints",
            }
            props = payload(await client.call_tool("props", {}))
            assert props["interpolation"]["speeds"]["slow"]["translation_m"] == 0.001
            response = await client.call_tool(
                "move_eef",
                {
                    "targets": {
                        "left": {"end_pos": [0.1, 0.03, 0.05, 0, 0, 0], "gripper": 1},
                        "right": {"end_pos": [0.05, 0, 0.1, 0, 0, 0], "gripper": 1},
                    },
                    "wait": True,
                    "timeout_s": 3,
                },
            )
            assert not response.is_error, payload(response)
            assert payload(response)["status"] == "completed"
            response = await client.call_tool(
                "state", {"include_images": True, "cameras": ["head"]}
            )
            state = payload(response)
            assert (
                len(state["qpos"])
                == len(state["qvel"])
                == len(state["effort"])
                == len(state["eef"])
                == 14
            )
            assert state["arms"]["left"]["end_pos"] == [0.1, 0.03, 0.05, 0, 0, 0]
            images = [block for block in response.content if block.type == "image"]
            assert len(images) == 1 and images[0].mime_type == "image/jpeg"
            assert Image.open(io.BytesIO(base64.b64decode(images[0].data))).format == "JPEG"
            home = await client.call_tool("send_command", {"commands": "both:home", "wait": True})
            assert not home.is_error and payload(home)["status"] == "completed"
            assert payload(home)["qpos"] == [0] * 14
            joint = await client.call_tool(
                "move_joints",
                {
                    "targets": {"left": {"joint_pos": [0.01, 0, 0, 0, 0, 0], "speed": "medium"}},
                    "wait": True,
                },
            )
            assert not joint.is_error and payload(joint)["qpos"][0] == 0.01

    with running_http() as (url, runtime):
        asyncio.run(scenario(url))


def test_http_invalid_batch_or_camera_does_not_move_robot():
    async def scenario(url):
        async with session(url) as client:
            invalid = await client.call_tool(
                "send_command",
                {
                    "commands": ["left:up", "right:fly"],
                    "clear_queue": True,
                },
            )
            assert invalid.is_error and not payload(invalid)["accepted"]
            invalid_camera = await client.call_tool(
                "send_command",
                {
                    "commands": "left:up",
                    "include_images": True,
                    "cameras": ["missing"],
                },
            )
            assert invalid_camera.is_error and not payload(invalid_camera)["accepted"]
            invalid_pose = await client.call_tool(
                "move_eef",
                {
                    "targets": {"left": {"end_pos": [0, 0, 0, 0, 0]}},
                },
            )
            assert invalid_pose.is_error
            state = payload(await client.call_tool("state", {}))
            assert state["recent_batches"] == []
            assert state["arms"]["left"]["end_pos"] == [0.2, 0.1, 0.2, 0, 0, 0]

    with running_http() as (url, runtime):
        asyncio.run(scenario(url))


def test_readonly_http_allows_observation_but_rejects_home():
    async def scenario(url):
        async with session(url) as client:
            state = await client.call_tool("state", {"include_images": True})
            assert not state.is_error and len(state.content) == 4
            assert not payload(state)["actuation_enabled"]
            home = await client.call_tool("send_command", {"commands": "both:home"})
            assert home.is_error and "disabled" in payload(home)["error"]

    with running_http(readonly=True) as (url, runtime):
        asyncio.run(scenario(url))


def test_readonly_ros_http_health_and_mcp_connection_create_no_command_publishers(monkeypatch):
    from test_observations_and_backends import fake_ros, put_feedback

    nodes = fake_ros(monkeypatch)
    with running_http(backend="ros", readonly=True) as (url, runtime):
        put_feedback(runtime.cache, "left")
        put_feedback(runtime.cache, "right")
        with httpx.Client(trust_env=False) as http:
            assert http.get(url + "/health").json()["actuation_enabled"] is False

        async def scenario():
            async with session(url) as client:
                await client.list_tools()
                await client.call_tool("props", {})
                state = await client.call_tool("state", {})
                assert len(payload(state)["qpos"]) == 14
                refused = await client.call_tool("send_command", {"commands": "both:home"})
                assert refused.is_error and "disabled" in payload(refused)["error"]

        asyncio.run(scenario())
        assert len(nodes) == 1 and nodes[0].publishers == []
        assert runtime.controller.progress()["recent_batches"] == []


def test_bearer_token_protects_http_and_sdk_client_can_authenticate():
    with running_http(token="test-secret") as (url, runtime):
        with httpx.Client(trust_env=False) as http:
            response = http.post(url + "/mcp", json={"jsonrpc": "2.0", "method": "ping", "id": 1})
            assert response.status_code == 401
            assert response.headers["www-authenticate"] == "Bearer"
            assert http.get(url + "/health").status_code == 401

        async def scenario():
            async with session(url, "test-secret") as client:
                assert payload(await client.call_tool("state", {}))["state_ready"]

        asyncio.run(scenario())


def test_untrusted_host_header_is_rejected():
    with running_http() as (url, runtime), httpx.Client(trust_env=False) as http:
        response = http.post(
            url + "/mcp",
            headers={"Host": "evil.example"},
            json={"jsonrpc": "2.0", "method": "ping", "id": 1},
        )
        assert response.status_code == 421


def test_robotics52_profile_accepts_remote_ip_mcp_client_and_rejects_other_hosts():
    root = Path(__file__).resolve().parents[1]
    config = load_config(root / "config/robotics52.yaml")
    with running_http(readonly=True, config=config) as (url, runtime):

        async def scenario():
            async with session(
                url,
                headers={"Host": "10.6.3.69:3000", "Origin": "http://10.6.3.69:3000"},
            ) as client:
                assert len((await client.list_tools()).tools) == 5
                state = payload(await client.call_tool("state", {}))
                assert state["state_ready"] and not state["actuation_enabled"]

        asyncio.run(scenario())
        with httpx.Client(trust_env=False) as http:
            response = http.post(
                url + "/mcp",
                headers={"Host": "evil.example"},
                json={"jsonrpc": "2.0", "method": "ping", "id": 1},
            )
            assert response.status_code == 421


def test_cli_check_config_does_not_require_ros_or_start_server(tmp_path):
    config = tmp_path / "config.yaml"
    config.write_text("http: {port: 3010}\ncontrol: {default_speed: medium}\n")
    root = Path(__file__).resolve().parents[1]
    env = {**os.environ, "PYTHONPATH": str(root / "src")}
    result = subprocess.run(
        [sys.executable, "-m", "ee_control_mcp", "--config", str(config), "--check-config"],
        cwd=root,
        env=env,
        text=True,
        capture_output=True,
    )
    assert result.returncode == 0, result.stderr
    data = json.loads(result.stdout)
    assert data["http"]["port"] == 3010 and data["control"]["default_speed"] == "medium"


def test_cli_invalid_config_exits_with_actionable_error(tmp_path):
    config = tmp_path / "invalid.yaml"
    config.write_text("control: {publish_rate_hz: -1}\n")
    root = Path(__file__).resolve().parents[1]
    env = {**os.environ, "PYTHONPATH": str(root / "src")}
    result = subprocess.run(
        [sys.executable, "-m", "ee_control_mcp", "--config", str(config), "--check-config"],
        cwd=root,
        env=env,
        text=True,
        capture_output=True,
    )
    assert result.returncode == 2
    assert "publish_rate_hz" in result.stderr and "Traceback" not in result.stderr


def test_cli_malformed_yaml_exits_with_actionable_error(tmp_path):
    config = tmp_path / "invalid.yaml"
    config.write_text("control: [\n")
    root = Path(__file__).resolve().parents[1]
    env = {**os.environ, "PYTHONPATH": str(root / "src")}
    result = subprocess.run(
        [sys.executable, "-m", "ee_control_mcp", "--config", str(config), "--check-config"],
        cwd=root,
        env=env,
        text=True,
        capture_output=True,
    )
    assert result.returncode == 2
    assert "YAML" in result.stderr and "Traceback" not in result.stderr
