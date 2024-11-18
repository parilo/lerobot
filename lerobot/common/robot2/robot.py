from abc import ABC, abstractmethod
import numpy as np
from dataclasses import dataclass


@dataclass
class MotorData:
    pos: np.ndarray
    vel: np.ndarray


class Robot(ABC):

    @abstractmethod
    def connect(self) -> None:
        ...

    @abstractmethod
    def disconnect(self) -> None:
        ...

    @abstractmethod
    def relax(self) -> None:
        ...

    @abstractmethod
    def read(self) -> MotorData:
        ...

    @abstractmethod
    def position_control(self, target_pos: np.ndarray) -> None:
        ...
