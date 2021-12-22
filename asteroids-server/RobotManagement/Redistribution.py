'''
Redistribute the robot
'''
from typing import List
import numpy as np
import scipy.optimize as optimize

class Redistribution:
    def __init__(self, track: np.ndarray) -> None:
        '''
        Track: an n by 2 array containg a series of discrete positions where the robots can stay 
        '''
        self._track = track

    def redistribute(self, numcam: int) -> np.ndarray:
        '''
        Arguments:
        - numcam: number of cameras (robots) to redistribute along the track
         
        Return:
        An numcam by 2 array of camera positions 
        '''
        if numcam == 0:
            print("Error! Trying to redistribute 0 positions in Redistribution.py/redistribute")
            return np.array([])

        pos_res = self._track[0]
        remaining_n = numcam - 1
        if remaining_n > 0:
            step = int((len(self._track) -  1) / remaining_n)
            indices = np.arange(start=1, stop=len(self._track), step=step)
            remaining_pos = self._track[indices]
            pos_res = np.concatenate((pos_res, remaining_pos), axis=0)

        return pos_res


    def assign(self, campos: np.ndarray, goalpos: np.ndarray) -> List[int]:
        """Assign robots to already computed positions

        Args:
            campos (np.ndarray): n by 2 array of current robot positions
            goalpos (np.ndarray): n by 2 array of target robot positions

        Returns:
            List[int]: indices into goalpos for each campos
        """

        cost_mat = np.apply_along_axis(lambda cam_pos: np.apply_along_axis(
            lambda goal_pos: np.linalg.norm(goal_pos - cam_pos), 1, goalpos),
            1, campos)
        row_ind, col_ind = optimize.linear_sum_assignment(cost_mat)
        return col_ind
