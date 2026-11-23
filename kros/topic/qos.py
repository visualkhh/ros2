from dataclasses import dataclass
from enum import Enum


class Durability(Enum):
    VOLATILE = "volatile"  # 지금 구독 중인 쪽에만 전달한다 (기본)
    TRANSIENT_LOCAL = "transient_local"  # 마지막 메시지를 기억해 뒀다가 나중에 구독한 쪽에게도 전달한다 (latch)


@dataclass(frozen=True)
class QoS:
    """토픽의 서비스 품질 (ROS의 QoS 중 지금 필요한 것만).

    depth:
      - 구독 쪽: 처리를 기다리는 메시지를 최대 몇 개까지 둘지. 넘치면 가장 오래된 것을 버린다 (Subscription.dropped로 센다).
        None이면 무제한 (기본: 아무것도 버리지 않는다).
      - 발행 쪽 + TRANSIENT_LOCAL: 나중에 구독한 쪽을 위해 기억해 둘 마지막 메시지 수.
    durability: 늦게 구독한 쪽이 이전 메시지를 받으려면 발행 쪽과 구독 쪽이 **둘 다** TRANSIENT_LOCAL이어야 한다 (ROS와 같은 규칙).

    주의: QOS_LATCHED(depth=1)를 **구독에** 쓰면 처리 대기 큐도 1개라서, 메시지가 연달아 오면 아직 처리 전인 오래된 것은 버려지고
    최신만 남는다 (ROS의 keep_last(1)과 같다). 하나도 놓치면 안 되면 구독에는 깊은 depth나 depth=None(무제한, 단 TRANSIENT_LOCAL은 depth 필요)을 쓴다.
    """

    depth: int | None = None
    durability: Durability = Durability.VOLATILE

    def __post_init__(self) -> None:
        if self.depth is not None and self.depth < 1:
            raise ValueError(f"QoS depth는 1 이상이어야 한다: {self.depth}")
        if self.durability is Durability.TRANSIENT_LOCAL and self.depth is None:
            raise ValueError("TRANSIENT_LOCAL은 몇 개를 기억할지 depth가 필요하다 (무제한으로 기억하지 않는다)")


QOS_DEFAULT = QoS()  # 무제한 큐, 휘발성: 지금까지의 동작
QOS_LATCHED = QoS(depth=1, durability=Durability.TRANSIENT_LOCAL)  # 마지막 하나를 기억 (tf_static, robot_description 같은 한 번만 내는 정보)
QOS_SENSOR_DATA = QoS(depth=5)  # 센서 값: 느린 구독자는 오래된 것을 건너뛰고 최신을 본다
QOS_COMMAND = QoS(depth=1)  # 명령(cmd_vel ...): 밀린 옛 명령은 버리고 최신만 (Pi가 바쁠 때 텔레옵 지연 방지)
