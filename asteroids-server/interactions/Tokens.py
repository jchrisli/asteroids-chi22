'''
Manage the tanible tokens used by the demonstrator
'''

from typing import List, Tuple
from enum import Enum
from math import hypot

from matplotlib.pyplot import close

class TokenType(Enum):
    SPOTLIGHT=1
    PIN=2
    NOTHING=100


class Tokens:
    _SEARCH_RANGE = 0.12
    _EXISTENCE_THRESHOLD = 60

    def __init__(self, spotlight_tag_id: int, spotlight_loc_tag_id: int, pin_tag_ids: List[int]) -> None:
        self._spotlight_tag_id = spotlight_tag_id
        self._pin_tag_ids = pin_tag_ids
        self._spotlight_loc_tag_id = spotlight_loc_tag_id
        self._spotlight_tag_pos = [1000.0, 1000.0]
        self._spotlight_loc_tag_pos = [-1.0, -1.0]
        self._spotlight_loc_counter = Tokens._EXISTENCE_THRESHOLD
        self._pin_tags_pos = {}
        for pin_tag_id in pin_tag_ids:
            self._pin_tags_pos[pin_tag_id] = [1000.0, 1000.0]

    def set_token_pos(self, tag_id: int, x: float, y: float) -> None:
        # print(f"Found token {tag_id}")
        if tag_id == self._spotlight_tag_id:
            self.set_spotlight_tag_pos(x, y)
        elif tag_id in self._pin_tag_ids:
            self._pin_tags_pos[tag_id] = [x, y]
        elif tag_id == self._spotlight_loc_tag_id:
            self._spotlight_loc_tag_pos = [x, y]
            self._spotlight_loc_counter = self._EXISTENCE_THRESHOLD

    def set_spotlight_tag_pos(self, x: float, y: float) -> None:
        self._spotlight_tag_pos = [x, y]

    def set_pin_tag_pos(self, tag_id: int, x: float, y: float) -> None:
        if tag_id in self._pin_tags_pos.keys():
            self._pin_tags_pos[tag_id] = [x, y]
        else:
            print(f"Error! Update unregistered pin tag {tag_id}")

    def get_spotlight_loc_pos(self) -> List[float]:
        return self._spotlight_loc_tag_pos

    @staticmethod
    def __close_enough(p1: List[float], p2: List[float]):
        print(f"Comparing {p1} with token position {p2}")
        return hypot(p1[0] - p2[0], p1[1] -  p2[1]) < Tokens._SEARCH_RANGE

    def search_for_tag(self, x: float, y: float) -> Tuple[int, TokenType]:
        p = [x, y]
        if Tokens.__close_enough(p, self._spotlight_tag_pos):
            return self._spotlight_tag_id, TokenType.SPOTLIGHT
        else:
            all_pin_tag_pos = self._pin_tags_pos.values()
            close_to_pin_tokens = list(map(lambda token_pos: Tokens.__close_enough(p, token_pos), all_pin_tag_pos))
            if any(close_to_pin_tokens):
                all_pin_tag_ids = list(self._pin_tags_pos.keys())
                close_token_ind = close_to_pin_tokens.index(True)
                return all_pin_tag_ids[close_token_ind], TokenType.PIN
            else:
                ## Other types of tokens
                return -1, TokenType.NOTHING

    def check_spotlight_loc_token_presence(self) -> bool:
        return self._spotlight_loc_tag_pos[0] > 0 and self._spotlight_loc_tag_pos[1] > 0

    def spotlight_loc_maybe_gone(self):
        self._spotlight_loc_counter -= 1
        if self._spotlight_loc_counter == 0:
            self._spotlight_loc_tag_pos = [-1.0, -1.0]

