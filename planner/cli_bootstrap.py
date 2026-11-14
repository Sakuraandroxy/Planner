"""Assemble the console runtime after the shared AirSim startup completes."""
from config.schema import PlannerConfig
from planner.adapters.airsim.connection import AirSimConnection
from planner.adapters.airsim.server_vehicle import ServerVehicle
from planner.bootstrap import Runtime, build_runtime


def build_cli_runtime(config: PlannerConfig, *, takeoff: bool) -> tuple[Runtime, ServerVehicle]:
    connection = AirSimConnection(config.airsim.host, config.airsim.port,
        config.airsim.connect_timeout_s, motion_timeout_s=config.airsim.move_timeout_s + 2)
    try:
        vehicle = ServerVehicle(connection, config.motion.limits, config.airsim.move_timeout_s,
            readiness_stable_s=config.server.readiness_stable_s,
            readiness_poll_interval_s=config.server.readiness_poll_interval_s,
            speed_mps=config.airsim.speed_mps, tracking=config.motion.continuous,
            velocity_command_duration_s=config.airsim.velocity_command_duration_s)
        connection.connect()
        vehicle.prepare(takeoff_if_landed=takeoff)
        runtime = build_runtime(config, connection=connection, holding=vehicle.holding)
        return runtime, vehicle
    except BaseException:
        # Match server startup: preserve the last hold goal if preparation fails.
        connection.close_transport()
        raise
