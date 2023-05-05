from polulu_command import polulu

class _Stop():
    def __init__(self) -> None:
        pass
    
    def go(self, pid):
        polulu.send_motor_command(pos=0, speed=0)
        polulu.send_servo_command(pos=0, speed=0)

stop = _Stop()