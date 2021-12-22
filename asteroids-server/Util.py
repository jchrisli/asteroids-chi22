## Various utilities 
# 

import math
from typing import Tuple, List
import numpy as np
from numpy import linalg

# From https://stackoverflow.com/a/7869457
def shortest_angular_dist(source: float, target: float) -> float:
    a = target - source
    return (a + math.pi) % (2 * math.pi) - math.pi

## Clip t to [a, b]
def clip(t: float, a: float, b:float) -> float:
    return min(b, max(t, a))

def normalize_vec(vec: np.ndarray):
    return vec / np.linalg.norm(vec)


def intersect_line_rect(p1: Tuple[float, float], p2: Tuple[float, float], rect_bl: Tuple[float, float],
                        rect_w: float, rect_h: float) -> List[np.ndarray]:
    p1v = np.array([p1[0], p1[1]])
    p2v = np.array([p2[0], p2[1]])

    dv = p2v - p1v
    dv = dv / np.linalg.norm(dv)

    top = rect_bl[1] + rect_h
    bottom = rect_bl[1]

    left = rect_bl[0]
    right = rect_bl[0] + rect_w

    intersections = []

    if dv[1] != 0:
        intersect_top_tvalue = (top - p1v[1]) / dv[1]
        intersect_top = p1v + intersect_top_tvalue * dv

        intersect_bottom_tvalue = (bottom - p1v[1]) / dv[1]
        intersect_bottom = p1v + intersect_bottom_tvalue * dv

        intersections.append(intersect_bottom)
        intersections.append(intersect_top)

    if dv[0] != 0:
        intersect_left_tvalue = (left - p1v[0]) / dv[0]
        intersect_left = p1v + intersect_left_tvalue * dv

        intersect_right_tvalue = (right - p1v[0]) / dv[0]
        intersect_right = p1v + intersect_right_tvalue * dv

        intersections.append(intersect_right)
        intersections.append(intersect_left)

    ## Find the intersections within the range
    tolerance = 0.01
    valid_insections = list(filter(lambda i: i[0] < right + tolerance and i[0] > left - tolerance and i[1] < top + tolerance and i[1] > bottom - tolerance, intersections))

    ## Rank the insections by their distance to p1
    if len(valid_insections) > 1:
        i0_dist_p1 = np.linalg.norm(valid_insections[0] - p1v)
        i1_dist_p1 = np.linalg.norm(valid_insections[1] - p1v)
        return [valid_insections[0], valid_insections[1]] if i0_dist_p1 < i1_dist_p1 else [valid_insections[1], valid_insections[0]]
    else: 
        return valid_insections

def get_rect_closest_side(p: Tuple[float, float], o: Tuple[float, float], w: float, h: float) -> int:
    # p_arr = np.array(p)[np.newaxis, :]
    to_left = abs(p[0] - o[0])
    to_right = abs(p[0] - (o[0] + w))
    to_top = abs(p[1] - o[1] - h)
    to_bottom = abs(p[1] - o[1])
    return int(np.argmin(np.array([to_bottom, to_right, to_top, to_left])))

def project_to_rect_side(p: Tuple[float, float], o: Tuple[float, float], w: float, h: float, side: int) -> Tuple[float, float]:
    if side == 0:
        return (p[0], o[1])
    elif side == 1:
        return (o[0] + w, p[1])
    elif side == 2:
        return (p[0], o[1] + h)
    else:
        return (o[0], p[1])

def get_rect_opposite_side(side: int) -> int:
    if side == 0:
        return 2
    elif side == 2:
        return 0
    elif side == 1:
        return 3
    else:
        return 1

