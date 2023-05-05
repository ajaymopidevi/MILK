from simple_pid import PID

class _PIDController:
    def __init__(self) -> None:
        self.pid = PID(0.18/2000, 0, 0, setpoint=0)

    def update(self, right_depth, left_depth, center_depth):
        current_control = self.pid(right_depth - left_depth)
        return current_control

pid_controller = _PIDController()
