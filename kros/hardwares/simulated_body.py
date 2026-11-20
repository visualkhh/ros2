from kros.hardwares.hardware import Hardware
from kros.worlds.objects.world_object import WorldObject
from kros.worlds.world import World


def simulated_body(hardware: Hardware) -> tuple[WorldObject, World]:
    """시뮬레이션 구현이 쓰는 도우미: 이 부품이 달린 몸체(WorldObject)와 그 몸체가 놓인 월드. 진짜 하드웨어 구현은 쓰지 않는다 (월드를 모른다)."""
    body = hardware.find_owner(WorldObject)  # type: ignore[type-abstract]  # 추상 클래스도 찾는 종류로 쓴다
    assert body is not None, f"{type(hardware).__name__}은 WorldObject(로봇) 아래에 달아야 한다"
    assert body.world is not None, f"{type(hardware).__name__}이 달린 {type(body).__name__}이 월드에 놓여 있지 않다 (World.add)"
    return body, body.world
