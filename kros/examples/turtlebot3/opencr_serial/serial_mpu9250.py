import dataclasses
import threading

from kros.hardwares.imus.mpu9250 import Mpu9250
from kros.msgs.sensor_msgs import Imu
from kros.msgs.std_msgs import Header
from kros.worlds.clock import Clock


class SerialMpu9250(Mpu9250):
    """OpenCR 내장 IMU: 보드가 보낸 I 줄의 값을 내보낸다. 보드는 20Hz로 보내므로 새 줄이 없는 주기에는 값을 내지 않는다 (같은 값을 다시 내보내지 않는다).
    부팅 때 자이로 바이어스를 보정하므로 켜고 ~2초는 가만히 둬야 한다 (펌웨어 쪽 사정)."""

    def __init__(self, frame_id: str = "imu_link", period: float | None = None) -> None:
        super().__init__(frame_id, period)
        self._lock = threading.Lock()
        self._pending: Imu | None = None

    def update(self, imu: Imu) -> None:
        """읽기 스레드가 새 I 줄을 넣는다."""
        with self._lock:
            self._pending = imu

    def sample(self, clock: Clock) -> Imu | None:
        with self._lock:
            imu, self._pending = self._pending, None
        return None if imu is None else dataclasses.replace(imu, header=Header(clock.time, self.frame_id))
