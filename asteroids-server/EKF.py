'''
Extended Kalman filter for our differential drive robot
Use velocity computed from DWA as the control input
'''
from typing import Tuple
import numpy as np

## ALL vectors are column vectors
class EKF:
    def __init__(self) -> None:
        # self._x = x0
        # self._P = P0
        # self._dt = 0.1
        ## formulation 1: [x, y, theta, v, omega]
        # self._proc_noise_cov = np.diag([0.01, 0.01, 0.001, 0.3, 0.3])
        # self._sens_noise_cov = np.diag([0.01, 0.01, 0.04])
        self._A1 = np.diag([1.0, 1.0, 1.0, 0, 0])
        self._A2 = np.diag([1.0, 1.0, 1.0])
        # A3 for Artifical potential field
        self._A3 = np.diag([1.0, 1.0])

        ## formulation 2: [x, y, theta]

    def predict(self, x: np.ndarray, u: np.ndarray, P: np.ndarray, Q: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        theta = x[2, 0]
        u0 = u[0, 0]
        u1 = u[1, 0]
        # B1 = dt * np.array([[np.cos(theta), 0],
        #                 [np.sin(theta), 0],
        #                 [0.0, 1.0],
        #                 [1.0, 0.0],
        #                 [0.0, 1.0]])
        # F1 = np.array([[1, 0, -np.sin(theta) * dt, 0, 0],
        #                 [0, 1, np.cos(theta) * dt, 0, 0],
        #                 [0, 0, 1, 0, 0],
        #                 [0, 0, 0, 0, 0],
        #                 [0, 0, 0, 0, 0]])
        B2 = dt * np.array([[np.cos(theta), 0],
                        [np.sin(theta), 0],
                        [0.0, 1.0]])
        F2 = np.array([[1, 0, -np.sin(theta) * dt * u0],
                        [0, 1, np.cos(theta) * dt * u0],
                        [0, 0, 1]])
        # Two formulations
        A = self._A2
        B = B2
        F = F3
        x_next = A @ x + B @ u
        P_next = F @ P @ F.T + Q * dt
        return x_next, P_next

    def __correct(self, x: np.ndarray, P: np.ndarray, z: np.ndarray, R: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        H1 = np.hstack((np.identity(3), np.zeros((3, 1)), np.zeros((3, 1))))
        H2 = np.identity(3)
        H = H2
        # The Kalman Gain!
        K = P @ H.T @ np.linalg.inv(H @ P @ H.T + R * dt)
        x_corrected = x + K @ (z - x)
        P_corrected = P - K @ H @ P
        return x_corrected, P_corrected

    def update(self, x: np.ndarray, u:np.ndarray, P: np.ndarray, z: np.ndarray, Q: np.ndarray, R: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        # Q = self._proc_noise_cov * dt
        # R = self._sens_noise_cov * dt

        x_pred, P_pred = self.predict(x, u, P, Q, dt)
        x_corr, P_corr = self.__correct(x_pred, P_pred, z, R, dt)
        return x_corr, P_corr