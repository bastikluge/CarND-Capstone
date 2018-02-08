from yaw_controller import YawController
from pid import PID
import math
import numpy as np

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.speed_controller = PID(0.22, 0.00009, 3.1, -5.0, 1.0)
        self.yaw_controller   = YawController(wheel_base, steer_ratio, ONE_MPH, max_lat_accel, max_steer_angle)
        pass

    def control(self, enabled, current_lin_v, current_ang_v, proposed_lin_v, proposed_ang_v):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not enabled:
            self.speed_controller.reset()
        dv = self.speed_controller.step(np.linalg.norm(current_lin_v - proposed_lin_v), 1.0/50.0)
        throttle = 0.0
        brake    = 0.0
        if dv < 0:
            brake = -dv; # @todo
        else:
            throttle = dv; # @todo
        steer = self.yaw_controller.get_steering(np.linalg.norm(proposed_lin_v), proposed_ang_v[2], np.linalg.norm(current_lin_v))
        return throttle, brake, steer
