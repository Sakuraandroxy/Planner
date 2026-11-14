"""Start the HTTP planner backend: python server.py --port 8080."""
import argparse
from dataclasses import replace
import logging
from pathlib import Path

from config import load_config
from planner.server_bootstrap import build_server


def main():
    parser = argparse.ArgumentParser(description="AirSim planner HTTP backend")
    parser.add_argument("--config", default=str(Path(__file__).parent / "config/base.yaml"))
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    args = parser.parse_args()
    if args.port is not None and not 1 <= args.port <= 65535:
        parser.error("port must be in 1..65535")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")
    config = load_config(args.config)
    settings = replace(config.server, host=args.host or config.server.host,
                       port=args.port if args.port is not None else config.server.port)
    config = replace(config, server=settings)
    import uvicorn
    uvicorn.run(build_server(config, args.config), host=settings.host, port=settings.port, workers=1)


if __name__ == "__main__":
    main()
