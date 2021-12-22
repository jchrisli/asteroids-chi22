'''
Track the states of the person (the demonstrator)
'''
from typing import List, Union
import numpy as np
from numpy.core.fromnumeric import argmin

class Person:
    _REACHABLE_DIST = 0.8
    def __init__(self) -> None:
        self._person_pos = None
        self._person_heading = None

    def update_person_state(self, pos: List[float], heading: float) -> None:
        self._person_pos = pos
        self._person_heading = heading

    def get_person_pos(self) -> Union[List[float], None]:
        return self._person_pos

    def get_person_heading(self) -> Union[float, None]:
        return self._person_heading

    def sample_reachable_points(self, num_sample=20) -> np.ndarray:
        if self._person_pos is not None:
            sample_pos_raw = np.random.uniform(-Person._REACHABLE_DIST, Person._REACHABLE_DIST, num_sample * 2)
            sample_pos = np.reshape(sample_pos_raw, (-1, 2))
            sample_pos = np.array(self._person_pos) + sample_pos
            return sample_pos
        else:
            return np.array([])

    def check_reachable(self, x: float, y: float) -> bool:
        if self._person_pos is not None:
            return np.hypot(x - self._person_pos[0], y -self._person_pos[1]) < self._REACHABLE_DIST
        else:
            return False
