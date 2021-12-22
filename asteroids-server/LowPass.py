'''
A low-pass filter for sensor measurements
Partly based on https://github.com/RobotLocomotion/drake-iiwa-driver/blob/61961b94e2ac097f0028b3cda177ad69ef5a944b/kuka-driver/low_pass_filter.h
Angle averaging using https://en.wikipedia.org/wiki/Mean_of_circular_quantities
'''
import math

class LowPass:
    def __init__(self, cutoff: float, dt: float, init_val: float, is_angle: bool = False) -> None:
        self._val = init_val
        rc = 1 / (2 * math.pi * cutoff)
        self._alpha = 1 - dt / (dt + rc)
        if self._alpha > 1 or self._alpha < 0:
            raise ValueError('Low pass filter alpha should be between 0 to 1. Check the parameters.')
        self._angle = is_angle

    def filter(self, input) -> float:
        val = self._val
        if not self._angle:
            val = val * self._alpha + input * (1 - self._alpha)
        else:
            sinm = math.sin(val) * self._alpha + math.sin(input) * (1 - self._alpha)
            cosm = math.cos(val) * self._alpha + math.cos(input) * (1 - self._alpha)
            val = math.atan2(sinm, cosm)
        self._val = val
        return self._val

    def force_update(self, update_to: float) -> float:
        self._val = update_to
        return self._val