"""Internal child-process entry point. The HTTP process never builds this runtime."""
import argparse
from dataclasses import asdict
import json
import logging
import sys
import threading

from config import load_config
from planner.adapters.airsim.connection import AirSimConnection
from planner.adapters.airsim.server_vehicle import ServerVehicle
from planner.application.backend_worker import serve_worker
from planner.domain.server import WorkerEvent, WorkerEventKind, WorkerMode


class RuntimeSession:
    def __init__(self, connection, config, runtime_factory):
        self.connection, self.config, self.runtime_factory = connection, config, runtime_factory
        self.runtime = None
        self.mode = None
        self.vehicle = ServerVehicle(connection, config.motion.limits, config.airsim.move_timeout_s,
            readiness_stable_s=config.server.readiness_stable_s,
            readiness_poll_interval_s=config.server.readiness_poll_interval_s,
            speed_mps=config.airsim.speed_mps, tracking=config.motion.continuous,
            velocity_command_duration_s=config.airsim.velocity_command_duration_s)

    @property
    def connection_lost(self):
        return self.connection.connection_lost

    def prepare(self, mode):
        self.mode = mode
        self.vehicle.prepare(takeoff_if_landed=mode is WorkerMode.PREPARE,
                             require_airborne=mode is WorkerMode.HOLD)
        if mode is not WorkerMode.CLEANUP:
            logging.getLogger(__name__).info("Vehicle hold confirmed. Assembling the task runtime.")
            self.runtime = self.runtime_factory(self.config, connection=self.connection, holding=self.vehicle.holding)

    def execute(self, instruction):
        return self.runtime.application.execute(instruction)

    def hold(self):
        self.vehicle.hold()

    def idle_tick(self):
        self.vehicle.holding.tick()

    def close(self):
        if self.runtime is not None:
            self.runtime.close()
        elif self.mode is WorkerMode.CLEANUP:
            self.connection.shutdown()
        else:
            # Failed preparation must not replace the last fixed hold goal.
            self.connection.close_transport()


def assemble_runtime(config, *, connection, holding):
    # Keep parser/workflow imports and construction after control handover.
    from planner.bootstrap import build_runtime
    return build_runtime(config, connection=connection, holding=holding)


class EventLogHandler(logging.Handler):
    def __init__(self, emit):
        super().__init__()
        self.emit_event = emit

    def emit(self, record):
        self.emit_event(WorkerEvent(WorkerEventKind.LOG, message=self.format(record), level=record.levelname))


def main():
    parser = argparse.ArgumentParser(description="Internal planner execution process")
    parser.add_argument("--config", required=True)
    parser.add_argument("--mode", choices=[m.value for m in WorkerMode], required=True)
    args = parser.parse_args()
    output_lock = threading.Lock()

    def emit(event):
        with output_lock:
            print(json.dumps(asdict(event), ensure_ascii=False), flush=True)

    from planner.adapters.process.command_inbox import CommandInbox
    inbox = CommandInbox(sys.stdin)

    logging.basicConfig(level=logging.INFO, handlers=[EventLogHandler(emit)], force=True)
    try:
        config = load_config(args.config)
        connection = AirSimConnection(config.airsim.host, config.airsim.port,
            config.airsim.connect_timeout_s, motion_timeout_s=config.airsim.move_timeout_s + 2)
    except Exception as exc:
        emit(WorkerEvent(WorkerEventKind.ERROR, message=f"Worker initialization failed: {exc}", level="ERROR"))
        return 1
    serve_worker(RuntimeSession(connection, config, assemble_runtime), WorkerMode(args.mode),
                 lambda: inbox.receive(), emit, idle_poll=inbox.receive)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
