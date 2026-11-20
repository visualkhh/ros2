from kros.msgs.sensor_msgs import Imu as ImuMessage
from kros.hardwares.hardware import Sensor


class Imu(Sensor[ImuMessage]):
    """IMU(관성 측정 장치) 인터페이스: 주기마다 방향, 각속도, 가속도를 담은 Imu 메시지를 내보낸다. read()/subscribe()로 받는다.
    값을 만드는 sample()은 모델/구현(시뮬레이션, 더미, 진짜)이 각자 한다.

    frame_id: 메시지 header에 적는 센서 좌표계 이름. 로봇의 URDF 링크 이름과 맞춰야 TF로 이어진다 (보통 imu_link).
    """

    def __init__(self, frame_id: str = "imu_link", period: float | None = None) -> None:
        super().__init__(period)
        self.frame_id = frame_id
