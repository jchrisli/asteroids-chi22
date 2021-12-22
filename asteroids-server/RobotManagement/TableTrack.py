'''
Generate the tracks used in Redistribution
'''
import numpy as np
from typing import List

def get_table_track(origin: List[float], w: float, h: float, padding: float = 0.1, numunit: int = 40):
    ## This is a hacky version for prototyping
    dist = (2 * (w - padding * 2) + 2 * (h - padding * 2)) / numunit
    br_points = [w -2 * padding, w - 2 * padding + h - 2 * padding, w + h + w - 6 * padding, w + h + w + h - 8 * padding]
    dist_sofar = 0
    points = []

    while dist_sofar < br_points[-1]:
        if dist_sofar < br_points[0]:
            points.append([origin[0] + padding + dist_sofar, origin[1] + padding])
        elif dist_sofar < br_points[1]:
            points.append([w + origin[0] - padding, dist_sofar - br_points[0] + origin[1] + padding])
        elif dist_sofar < br_points[2]:
            points.append([origin[0] + padding + br_points[2] - dist_sofar, origin[1] + h - padding])
        else:
            points.append([origin[0] + padding, origin[1] + padding + br_points[3] - dist_sofar])
        dist_sofar += dist
    
    return np.array(points)
