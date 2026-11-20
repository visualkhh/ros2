from kros.hardwares.motors.motor import Motor
from kros.worlds.clock import Clock


class SerialWheel(Motor):
    """OpenCR에 달린 바퀴 모터 한 개 (진짜 XM430-W210). position/velocity는 보드가 보낸 S 줄의 값이고, 목표 속도는 보드가 두 바퀴를 묶어 C 줄 하나로 보낸다.
    그래서 apply()는 하는 일이 없다: write()로 넣은 목표는 OpenCrSerial이 가져가 보낸다."""

    model = "XM430-W210"

    def apply(self, command: float, clock: Clock) -> None:
        pass
