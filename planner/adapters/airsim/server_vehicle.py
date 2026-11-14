"""Simulator readiness and fixed-position hold on the sole execution thread."""
import logging
import math
import time

from planner.errors import ExecutionError, SimulatorConnectionError
from planner.domain.motion import ContinuousTrackingSettings
from planner.adapters.airsim.vehicle import AirSimVehicle
from planner.services.position_hold import PositionHold

logger = logging.getLogger(__name__)


class ServerVehicle:
    def __init__(self, connection, limits, timeout_s, *, readiness_stable_s=2.,
                 readiness_poll_interval_s=.25,
                 speed_mps=2., tracking=ContinuousTrackingSettings(),
                 velocity_command_duration_s=.2,
                 clock=time.monotonic, sleep=time.sleep):
        self.connection, self.limits, self.timeout_s = connection, limits, timeout_s
        self.readiness_stable_s = readiness_stable_s
        self.readiness_poll_interval_s = readiness_poll_interval_s
        self.clock, self.sleep = clock, sleep
        self.control = AirSimVehicle(connection, speed_mps, timeout_s, limits,
                                    velocity_command_duration_s=velocity_command_duration_s)
        self.holding = PositionHold(self.control, limits, tracking, speed_mps, timeout_s, clock=clock, sleep=sleep)
        self.control.holding = self.holding

    def _ping(self):
        if self.connection.call("ping") is not True:
            raise SimulatorConnectionError("AirSim ping failed")

    @staticmethod
    def _position(state):
        p = state.kinematics_estimated.position
        position = (p.x_val, p.y_val, p.z_val)
        if not all(math.isfinite(v) for v in position):
            raise ExecutionError("AirSim returned a non-finite hold position")
        return position

    def _wait_ready(self):
        import airsim
        began, stable_since = self.clock(), None
        previous_timestamp, previous_landed = None, None
        logger.info("Waiting for advancing AirSim state (stable period %.2fs).", self.readiness_stable_s)
        while self.clock() - began < self.timeout_s:
            self._ping()
            state = self.connection.call("getMultirotorState")
            self._position(state)
            v = state.kinematics_estimated.linear_velocity
            quiet = (state.landed_state != airsim.LandedState.Landed or
                     math.hypot(v.x_val, v.y_val, v.z_val) <= self.limits.stopped_speed_mps)
            advancing = (previous_timestamp is not None and state.timestamp > previous_timestamp and
                         state.landed_state == previous_landed)
            now = self.clock()
            stable_since = (now if stable_since is None else stable_since) if advancing and quiet else None
            if stable_since is not None and now - stable_since >= self.readiness_stable_s:
                return state
            previous_timestamp, previous_landed = state.timestamp, state.landed_state
            self.sleep(self.readiness_poll_interval_s)
        raise ExecutionError("AirSim state did not become ready before the startup timeout")

    def prepare(self, *, takeoff_if_landed: bool, require_airborne: bool = False):
        import airsim
        if takeoff_if_landed:
            state = self._wait_ready()
            landed = state.landed_state == airsim.LandedState.Landed
            self.connection.prepare_vehicle(arm=landed)
            if landed:
                logger.info("AirSim ready; starting takeoff.")
                initial_z = self._position(state)[2]
                self.connection.call("cancelLastTask")
                takeoff_error = None
                try:
                    self.connection.takeoff(timeout_s=self.timeout_s, hover_after=False)
                except ExecutionError as exc:
                    takeoff_error = exc
                state = self.connection.call("getMultirotorState")
                climb = initial_z-self._position(state)[2]
                logger.info("Takeoff result: landed_state=%s climb=%.3fm initial_z=%.3f actual_z=%.3f native_confirmed=%s",
                            state.landed_state, climb, initial_z, self._position(state)[2], takeoff_error is None)
                if state.landed_state == airsim.LandedState.Landed:
                    raise ExecutionError("Takeoff returned but the vehicle is still landed")
                if climb <= self.limits.position_tolerance_m:
                    raise ExecutionError("Takeoff did not produce a measured climb from the ground position") from takeoff_error
                if takeoff_error is not None:
                    logger.warning("Native takeoff was not confirmed; measured climb exists. Position hold must confirm readiness.")
            self.hold(require_airborne=True)
        else:
            self._ping()
            # Reusing enabled API control must not reset the SimpleFlight goal.
            # Airborne stop/cleanup never arms or initiates another takeoff.
            self.connection.prepare_vehicle(arm=False)
            self.hold(require_airborne=require_airborne)

    def hold(self, *, require_airborne: bool = True):
        import airsim
        state = self.connection.call("getMultirotorState")
        if state.landed_state == airsim.LandedState.Landed:
            if require_airborne:
                raise ExecutionError("Cannot confirm airborne hold: vehicle is landed")
            self.connection.call("cancelLastTask")
            logger.info("Vehicle is already landed; no takeoff requested.")
            return
        if not self.holding.active:
            self.holding.start()
        self.holding.wait()
