
from typing import List


class DataLogger:
    def __init__(self, robot_motion_logs: str, interaction_logs: str, workspace_logs: str) -> None:
        self._motion_log_path = robot_motion_logs
        self._inter_log_path = interaction_logs
        self._workspace_log_path = workspace_logs
        try:
            self._motion_log_f = open(self._motion_log_path, 'w')
            self._inter_log_f = open(self._inter_log_path, 'w')
            self._workspace_log_f = open(self._workspace_log_path, 'w')
        except IOError:
            print(f"Cannot open data log file {self._motion_log_path} or {self._inter_log_path} or {self._workspace_log_path}")
            exit()

    def write_motion_log(self, ts: float, bot_id: int, x: float, y: float, h: float, users: List[str] ) -> None:
        motion_line = f"{ts}:{bot_id}:{x}:{y}:{h}:{users}"
        self._motion_log_f.write(f"{motion_line}\n")

    def write_interaction_log(self, ts:float, user:str, command_type: str, command_params: List[float]) -> None:
        interaction_line = f"{ts}:{user}:{command_type}:{command_params}"
        self._inter_log_f.write(f"{interaction_line}\n")

    def write_workspace_log(self, ts: float, workspace_id: float, x1: float, y1: float, x2: float, y2: float) -> None:
        workspace_line = f"{ts}:{workspace_id}:{x1}:{y1}:{x2}:{y2}"
        self._workspace_log_f.write(f"{workspace_line}\n")

    def close(self):
        self._motion_log_f.close()
        self._inter_log_f.close()
        self._workspace_log_f.close()

