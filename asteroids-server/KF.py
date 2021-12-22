'''
Linear Kalman filter, only tracking positions
Based on https://www.intechopen.com/books/introduction-and-implementations-of-the-kalman-filter/introduction-to-kalman-filter-and-its-applications
'''

from typing import Tuple
import numpy as np

class KF:
    def __init__(self) -> None:
        self._F = np.diag([1.0, 1.0])
        self._H = np.diag([1.0, 1.0])
        
    ## Maybe should use the average of current theta and the target theta
    def predict(self, x: np.ndarray, u: float, P: np.ndarray, Q: np.ndarray, theta: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        B = np.array([[np.cos(theta)], [np.sin(theta)]]) * dt
        x_pred = self._F @ x + B * u
        P_pred = self._F @ P @ self._F.T + Q
        return x_pred, P_pred

