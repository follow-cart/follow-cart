from enum import Enum

class VelocityCalculator:
    ERROR = 1e-3

    def __init__(self, rate: float,duration: float, first_value: float, target_value: float):
        self.rate = rate
        self.duration = duration
        self.first_value = first_value
        self.target_value = target_value
        self.current_value = first_value
        self.step_value = (self.target_value - self.first_value) / (self.rate * self.duration)
        self._state = VelocityCalculator.State.INIT

    def next_value(self) -> float:
        if abs(self.target_value - self.current_value) < VelocityCalculator.ERROR:
            self._state = VelocityCalculator.State.REACHED
            return self.target_value

        self.current_value = self.current_value + self.step_value
        self._state = VelocityCalculator.State.CHANGED
        return self.current_value

    @property
    def state(self):
        return self._state

    def set_reached(self):
        self._state = VelocityCalculator.State.REACHED
        
    class State(Enum):
        INIT = 0
        CHANGED = 1
        REACHED = 2
