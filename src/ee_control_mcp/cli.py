from __future__ import annotations

import argparse
import json
import logging
from dataclasses import replace

from .config import load_config


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Dual-arm ROS 2 HTTP MCP server")
    parser.add_argument("--config", help="YAML config; omitted uses built-in defaults")
    parser.add_argument("--backend", choices=("ros", "sim"), default="ros")
    parser.add_argument(
        "--no-actuation", action="store_true", help="read observations without command publishers"
    )
    parser.add_argument(
        "--check-config",
        action="store_true",
        help="validate/print config without connecting to ROS",
    )
    parser.add_argument("--host", help="HTTP bind address override")
    parser.add_argument("--port", type=int, help="HTTP port override")
    parser.add_argument(
        "--log-level", choices=("debug", "info", "warning", "error"), default="info"
    )
    args = parser.parse_args(argv)
    try:
        config = load_config(args.config)
        if args.port is not None and not 1 <= args.port <= 65535:
            raise ValueError("port must be in [1, 65535]")
        config = replace(
            config,
            http=replace(
                config.http,
                host=args.host or config.http.host,
                port=config.http.port if args.port is None else args.port,
            ),
        )
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    if args.check_config:
        print(json.dumps(config.as_dict(), indent=2, ensure_ascii=False))
        return 0

    import uvicorn

    from .runtime import Runtime
    from .server import create_app

    logging.basicConfig(level=args.log_level.upper(), format="%(levelname)s %(name)s: %(message)s")
    runtime = Runtime(config, backend=args.backend, actuation_enabled=not args.no_actuation)
    app = create_app(runtime)
    uvicorn.run(
        app,
        host=config.http.host,
        port=config.http.port,
        log_level=args.log_level,
        workers=1,
        timeout_graceful_shutdown=5,
        ws="none",
    )
    return 0
