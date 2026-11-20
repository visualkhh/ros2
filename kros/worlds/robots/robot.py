from abc import ABC
from pathlib import Path
from typing import Any

from kros.msgs.geometry_msgs import Twist
from kros.hardwares import Hardware
from kros.interactions import Interaction
from kros.worlds.objects.world_object import WorldObject


class Robot(Hardware, WorldObject, ABC):
    """로봇: 부품(모터, 센서)을 가진 Hardware이면서 월드에 놓이는 WorldObject. 월드의 시계 신호를 부품에게 내려준다. kros와 무관하다."""

    name: str
    urdf_path: Path  # URDF 파일
    description_dir: Path  # URDF의 package:// 경로가 가리키는 폴더
    radius: float  # m, 충돌 판정용 원 반지름

    command: Twist  # 로봇이 받은 속도 명령 (바깥에서 넣는다)

    def joint_positions(self) -> dict[str, float]:
        """움직이는 관절의 현재 위치 (URDF 관절 이름 -> rad 또는 m). 바퀴 모터 엔코더처럼 하드웨어가 남긴 사실이다. 기본은 없음."""
        return {}

    def receive(self, interaction: Interaction[Any]) -> None:
        """사람의 입력 장치(키보드, 게임패드 ...)를 받는다. 입력이 들어올 때마다 on_interaction 훅이 불린다."""
        interaction.subscribe(lambda state: self.on_interaction(interaction, state))

    def on_interaction(self, interaction: Interaction[Any], state: Any) -> None:
        """훅: 받은 입력 장치에 입력이 들어왔다. 무슨 뜻으로 해석할지는 로봇이 정한다 (기본은 무시)."""
