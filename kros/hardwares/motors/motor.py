from kros.hardwares.hardware import Actuator


class Motor(Actuator[float]):
    """회전 모터 한 개의 인터페이스: write(목표 속도 rad/s)로 명령을 내리고, position(누적 회전 rad)과 velocity(현재 속도 rad/s)를 읽는다.
    바퀴에도 매니퓰레이터 관절에도 쓴다. 실행(apply)은 모델/구현(시뮬레이션, 진짜)이 한다."""

    def __init__(self, period: float | None = None) -> None:
        super().__init__(0.0, period)
        self.position = 0.0  # rad, 누적 회전각 (엔코더)
        self.velocity = 0.0  # rad/s, 현재 속도
