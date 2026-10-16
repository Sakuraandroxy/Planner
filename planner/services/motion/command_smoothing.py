"""Stateful, vector-norm acceleration/jerk limits for streaming commands."""
import math


def bounded(vector, maximum):
    norm = math.hypot(*vector)
    scale = min(1., maximum/norm) if norm else 1.
    return tuple(x*scale for x in vector)


class CommandSmoother:
    def __init__(self, acceleration, jerk):
        self.maximum_acceleration, self.maximum_jerk = acceleration, jerk
        self.bandwidth = min(4., 2*math.sqrt(jerk/acceleration))
        self.reset()

    def reset(self, velocity=(0., 0., 0.)):
        self.velocity = tuple(velocity)
        self.acceleration = tuple(0. for _ in velocity)

    def step(self, desired, dt):
        # Critically damped velocity response: its acceleration is itself a
        # state, rather than an instantaneous clipped velocity difference.
        w = self.bandwidth
        jerk = bounded(tuple(w*w*(goal-v)-2*w*a for goal, v, a in
                             zip(desired, self.velocity, self.acceleration)), self.maximum_jerk)
        previous = self.acceleration
        acceleration = bounded(tuple(a+j*dt for a, j in zip(previous, jerk)), self.maximum_acceleration)
        # Projection onto the acceleration ball is nonexpansive, so it cannot
        # increase the jerk-limited change from an already admissible state.
        self.velocity = tuple(v+(a+b)*.5*dt for v, a, b in zip(self.velocity, previous, acceleration))
        self.acceleration = acceleration
        return self.velocity
