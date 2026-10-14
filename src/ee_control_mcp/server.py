from __future__ import annotations

import asyncio
import base64
import hmac
import json
import os
from contextlib import asynccontextmanager
from typing import Annotated

from mcp.server import MCPServer
from mcp.server.transport_security import TransportSecuritySettings
from mcp.types import CallToolResult, ImageContent, TextContent, ToolAnnotations
from pydantic import BaseModel, ConfigDict, Field, FiniteFloat, field_validator
from starlette.applications import Starlette
from starlette.datastructures import Headers
from starlette.responses import JSONResponse
from starlette.routing import Mount, Route

from . import __version__
from .commands import MOTIONS, parse_commands, structured_step
from .observations import EncodedImage
from .recording import ObservationRecorder
from .runtime import Runtime
from .types import Arm, Speed, finite_number, finite_vector

Timeout = Annotated[float, Field(ge=0, le=120)]
Quality = Annotated[int, Field(ge=1, le=100)]
CameraNames = Annotated[list[str], Field(min_length=1)]
Vector6 = Annotated[list[FiniteFloat], Field(min_length=6, max_length=6)]


class TargetOptions(BaseModel):
    model_config = ConfigDict(extra="forbid")
    gripper: FiniteFloat | None = None
    speed: Speed | None = None

    @field_validator("gripper", mode="before")
    @classmethod
    def validate_gripper(cls, value):
        return None if value is None else finite_number(value, "gripper")


class EefTarget(TargetOptions):
    end_pos: Vector6

    @field_validator("end_pos", mode="before")
    @classmethod
    def validate_pose(cls, value):
        return list(finite_vector(value, 6, "end_pos"))


class JointTarget(TargetOptions):
    joint_pos: Vector6

    @field_validator("joint_pos", mode="before")
    @classmethod
    def validate_joints(cls, value):
        return list(finite_vector(value, 6, "joint_pos"))


class BearerTokenMiddleware:
    def __init__(self, app, token: str):
        self.app = app
        self._expected = f"Bearer {token}".encode()

    async def __call__(self, scope, receive, send):
        if scope["type"] == "http":
            supplied = Headers(scope=scope).get("authorization", "").encode()
            if not hmac.compare_digest(supplied, self._expected):
                response = JSONResponse(
                    {"error": "unauthorized"},
                    status_code=401,
                    headers={"WWW-Authenticate": "Bearer"},
                )
                await response(scope, receive, send)
                return
        await self.app(scope, receive, send)


def _result(payload: dict, images: list[EncodedImage] = (), *, error=False) -> CallToolResult:
    content = [
        TextContent(type="text", text=json.dumps(payload, ensure_ascii=False, allow_nan=False))
    ]
    content.extend(
        ImageContent(
            type="image", mime_type="image/jpeg", data=base64.b64encode(frame.data).decode("ascii")
        )
        for frame in images
    )
    return CallToolResult(content=content, structured_content=payload, is_error=error)


def create_app(runtime: Runtime, *, token: str | None = None) -> Starlette:
    config = runtime.config
    recorder = ObservationRecorder()
    server = MCPServer(
        "ee-control-mcp",
        version=__version__,
        instructions=(
            "Control a ROS 2 dual-arm robot. Read props for directions, units and interpolation "
            "settings, and state for current feedback. move_eef targets are absolute xyz/rpy "
            "in meters/radians; send_command moves relative commanded targets. home zeros six "
            "joints and closes the gripper. Control calls share a serial queue. Images are "
            "optional JPEG blocks; their camera names and timestamps are in the JSON result."
        ),
    )
    readonly = ToolAnnotations(
        read_only_hint=True, destructive_hint=False, idempotent_hint=True, open_world_hint=False
    )
    control = ToolAnnotations(
        read_only_hint=False, destructive_hint=True, idempotent_hint=False, open_world_hint=False
    )

    def validate_images(cameras, quality):
        if cameras is not None and (
            not cameras
            or len(set(cameras)) != len(cameras)
            or any(name not in config.ros.camera_topics for name in cameras)
        ):
            raise ValueError(f"cameras must be unique names from {tuple(config.ros.camera_topics)}")
        if quality is not None and not 1 <= quality <= 100:
            raise ValueError("image_quality must be in [1, 100]")

    def attach_images(payload, include_images, cameras, quality):
        if not include_images:
            return payload, []
        metadata, images = runtime.cache.images(
            cameras,
            max_age_s=config.observation.image_max_age_s,
            quality=quality if quality is not None else config.observation.image_quality,
            max_width=config.observation.image_max_width,
            max_height=config.observation.image_max_height,
        )
        return {**payload, **metadata}, images

    async def observation_result(payload, images=(), *, error=False):
        log_errors = await asyncio.to_thread(recorder.record, payload, images)
        if log_errors:
            payload = {**payload, "observation_log_errors": log_errors}
        return _result(payload, images, error=error)

    @server.tool(annotations=readonly)
    def props() -> CallToolResult:
        """Read ROS topics, raw state layouts, directions and uniform interpolation settings."""
        return _result(
            {
                "backend": runtime.backend.name,
                "connected": runtime.backend.connected,
                "actuation_enabled": runtime.backend.actuation_enabled,
                "topics": {
                    "commands": config.ros.command_topics,
                    "feedback": config.ros.feedback_topics,
                    "cameras": config.ros.camera_topics,
                },
                "state_legend": {
                    "qpos": "[left J1..J6, left raw gripper, right J1..J6, right raw gripper]",
                    "qvel": "left joint_vel[7] followed by right joint_vel[7]",
                    "effort": (
                        "left joint_cur[7] followed by right joint_cur[7]; driver current units"
                    ),
                    "eef": (
                        "[left x,y,z,roll,pitch,yaw,raw gripper, "
                        "right x,y,z,roll,pitch,yaw,raw gripper]"
                    ),
                    "end_pos_units": "meters, then xyz Euler angles in radians",
                    "gripper": (
                        "feedback is raw; configured scale/offset converts it to command units"
                    ),
                },
                "vocabulary": {
                    "arms": ["left", "right", "both"],
                    "motions": list(MOTIONS),
                    "grammar": "<arm>:<motion>[&motion][:value][@slow|medium|fast]",
                    "step_join": "|",
                    "motion_deltas": config.control.motions,
                    "delta_legend": ["dx", "dy", "dz", "drx", "dry", "drz"],
                    "rotation_frame": "local end-effector frame; rotation-vector composition",
                    "translation_frame": "robot base frame",
                    "home": {"mode": 5, "joint_pos": [0] * 6, "gripper": 0},
                },
                "interpolation": {
                    "method": (
                        "uniform linear translation/joints and shortest-arc SLERP orientation"
                    ),
                    "publish_rate_hz": config.control.publish_rate_hz,
                    "default_speed": config.control.default_speed,
                    "speeds": {
                        speed: {"translation_m": value.translation_m, "angle_rad": value.angle_rad}
                        for speed, value in config.control.speeds.items()
                    },
                },
                "control": config.as_dict()["control"],
                "observation": config.as_dict()["observation"],
            }
        )

    @server.tool(annotations=readonly)
    async def state(
        include_images: bool = False,
        cameras: CameraNames | None = None,
        image_quality: Quality | None = None,
        wait_for_new: bool = False,
        since_seq: Annotated[int, Field(ge=0)] | None = None,
        timeout_s: Timeout = 3,
    ) -> CallToolResult:
        """Read raw ROS feedback and queue progress, optionally attaching named JPEG camera frames.

        Reading never consumes messages. wait_for_new waits for newer arm feedback than since_seq;
        omit since_seq to wait for an update after this call starts. Missing/stale/corrupt images
        are reported separately with camera names and image timestamps.
        """
        try:
            validate_images(cameras, image_quality)
            waited = False
            if wait_for_new:
                since = runtime.cache.seq if since_seq is None else since_seq
                waited = await asyncio.to_thread(runtime.cache.wait_for_new, since, timeout_s)
            payload = {**runtime.snapshot(), "waited_for_new": waited}
            payload, images = await asyncio.to_thread(
                attach_images, payload, include_images, cameras, image_quality
            )
            return await observation_result(payload, images)
        except (ValueError, RuntimeError) as exc:
            return _result({"error": str(exc)}, error=True)

    async def call_control(
        make_steps, wait, timeout_s, clear_queue, include_images, cameras, image_quality
    ):
        try:
            # Validate every option before any robot-side effect, including camera selection.
            validate_images(cameras, image_quality)
            steps = make_steps()
            response = await asyncio.to_thread(
                runtime.controller.submit, steps, clear_queue=clear_queue
            )
            if wait:
                response = await asyncio.to_thread(
                    runtime.controller.wait, response["batch_id"], timeout_s
                )
            payload = {**runtime.snapshot(), **response, "waited": wait}
            payload, images = await asyncio.to_thread(
                attach_images, payload, include_images, cameras, image_quality
            )
            return await observation_result(
                payload, images, error=response["status"] in ("failed", "cancelled")
            )
        except (ValueError, RuntimeError) as exc:
            return await observation_result(
                {**runtime.snapshot(), "accepted": False, "error": str(exc)}, error=True
            )

    @server.tool(annotations=control)
    async def send_command(
        commands: str | list[str | list[str]],
        wait: bool = False,
        timeout_s: Timeout = 30,
        clear_queue: bool = False,
        include_images: bool = False,
        cameras: CameraNames | None = None,
        image_quality: Quality | None = None,
    ) -> CallToolResult:
        """Queue a validated chunk of relative arm commands.

        Grammar: left|right|both:motion[&motion][:value][@slow|medium|fast]. Motions:
        forward/backward (+x/-x), left/right (+y/-y), up/down (+z/-z); twist-clockwise/
        twist-counterclockwise (+rx/-rx), tilt-down-toward-table/tilt-up-away-from-table
        (+ry/-ry), turn-left-across-table/turn-right-across-table (+rz/-rz); open, close,
        hold, home. props reports calibrated direction overrides. Distance/angle is positive,
        in meters/radians; open/close explicit value is a finite absolute command in native
        RobotCmd gripper units (may be negative). Omitted gripper values use props configuration.
        A shared value applies to every motion in an & group, including its gripper word.

        Examples: 'left:up:0.02@slow | right:hold'; [['left:up', 'right:open'], 'both:home'].
        Parallel clauses cannot overlap arms. Moves accumulate on commanded targets. home
        sets mode=5, joints=0 and gripper=0. clear_queue drops pending steps, preserving the
        active step. wait waits for THIS batch; a waiting timeout does not cancel motion.
        """
        return await call_control(
            lambda: parse_commands(commands, config.control.default_speed),
            wait,
            timeout_s,
            clear_queue,
            include_images,
            cameras,
            image_quality,
        )

    @server.tool(annotations=control)
    async def move_eef(
        targets: dict[Arm, EefTarget],
        wait: bool = False,
        timeout_s: Timeout = 30,
        clear_queue: bool = False,
        include_images: bool = False,
        cameras: CameraNames | None = None,
        image_quality: Quality | None = None,
    ) -> CallToolResult:
        """Queue absolute END_CONTROL (mode=4) targets for left and/or right in one parallel step.

        Each target has end_pos=[x,y,z,roll,pitch,yaw] in meters/radians, optional gripper in
        RobotCmd units and optional speed=slow|medium|fast. Omitted gripper preserves the
        previous command, initially calibrated feedback. Both arms may use different goals
        and speeds. Uniform interpolation, feedback arrival and wait semantics match send_command.
        """
        return await call_control(
            lambda: [
                structured_step(
                    4,
                    {arm: target.model_dump(exclude_none=True) for arm, target in targets.items()},
                    config.control.default_speed,
                )
            ],
            wait,
            timeout_s,
            clear_queue,
            include_images,
            cameras,
            image_quality,
        )

    @server.tool(annotations=control)
    async def move_joints(
        targets: dict[Arm, JointTarget],
        wait: bool = False,
        timeout_s: Timeout = 30,
        clear_queue: bool = False,
        include_images: bool = False,
        cameras: CameraNames | None = None,
        image_quality: Quality | None = None,
    ) -> CallToolResult:
        """Queue absolute POSITION_CONTROL (mode=5) joint targets for left and/or right.

        Each target has joint_pos=[J1..J6] in radians (exactly six), optional gripper in
        RobotCmd units and speed=slow|medium|fast. Both arms execute in one parallel step;
        omitted gripper is preserved. The gripper is separate from the six joint angles.
        """
        return await call_control(
            lambda: [
                structured_step(
                    5,
                    {arm: target.model_dump(exclude_none=True) for arm, target in targets.items()},
                    config.control.default_speed,
                )
            ],
            wait,
            timeout_s,
            clear_queue,
            include_images,
            cameras,
            image_quality,
        )

    mcp_app = server.streamable_http_app(
        stateless_http=True,
        json_response=True,
        transport_security=TransportSecuritySettings(
            allowed_hosts=list(config.http.allowed_hosts),
            allowed_origins=list(config.http.allowed_origins),
        ),
    )

    @asynccontextmanager
    async def lifespan(app):
        await asyncio.to_thread(runtime.start)
        try:
            async with server.session_manager.run():
                yield
        finally:
            await asyncio.to_thread(runtime.close)

    async def health(request):
        return JSONResponse(
            {
                "backend": runtime.backend.name,
                "connected": runtime.backend.connected,
                "state_ready": runtime.cache.state(config.observation.feedback_max_age_s)[
                    "state_ready"
                ],
                "actuation_enabled": runtime.backend.actuation_enabled,
            }
        )

    app = Starlette(routes=[Route("/health", health), Mount("/", app=mcp_app)], lifespan=lifespan)
    configured_token = os.environ.get(config.http.token_env) if token is None else token
    if configured_token:
        app.add_middleware(BearerTokenMiddleware, token=configured_token)
    return app
