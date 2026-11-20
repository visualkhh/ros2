from collections.abc import Callable

from kros.diagnostics import DiagnosticStatusWrapper
from kros.msgs.diagnostic_msgs import DiagnosticLevel
from kros.worlds.hardwares import Actuator, Hardware, Sensor


def hardware_path(hardware: Hardware) -> str:
    """로봇부터 이 장치까지 이름을 이은 경로 ("turtlebot3_waffle_pi/drive/left_wheel")."""
    names: list[str] = []
    node: Hardware | None = hardware
    while node is not None:
        names.append(node.name or type(node).__name__)
        node = node.owner
    return "/".join(reversed(names))


def leaves(hardware: Hardware) -> list[Hardware]:
    """장치와 그 부품들 중 센서/구동기인 것들 (트리를 따라 모두)."""
    found: list[Hardware] = [hardware] if isinstance(hardware, (Sensor, Actuator)) else []
    for part in hardware.parts:
        found.extend(leaves(part))
    return found


def make_task(hardware: Hardware, now: Callable[[], float], data_timeout: Callable[[], float]) -> Callable[[DiagnosticStatusWrapper], None]:
    """하드웨어가 남긴 사실(전원, 실패 횟수, 최신 값의 시각)을 진단 수준으로 바꾸는 판단. 판단은 쓰는 쪽(여기)이 한다.

    - 전원이 꺼져 있다 -> WARN
    - 센서가 한 번도 값을 못 냈다 -> WARN ("아직 값이 없다")
    - 센서의 최신 값이 data_timeout보다 오래됐다 -> ERROR
    - 지난 진단 이후 새 실패가 있다 -> WARN
    - 그 밖에는 OK
    """
    seen_errors = 0  # 지난 진단 때까지 센 실패 횟수

    def task(status: DiagnosticStatusWrapper) -> None:
        nonlocal seen_errors
        new_errors = hardware.error_count - seen_errors
        seen_errors = hardware.error_count
        status.add("model", hardware.model or "-")
        status.add("powered", hardware.is_on)
        status.add("period", hardware.period if hardware.period is not None else "every tick")
        status.add("error_count", hardware.error_count)
        status.add("last_error", hardware.last_error if hardware.last_error is not None else "-")
        problems: list[tuple[DiagnosticLevel, str]] = []  # 지금 보이는 문제들 (수준, 설명)
        if isinstance(hardware, Sensor):
            age = None if hardware.stamp is None else now() - hardware.stamp
            status.add("data_age", f"{age:.2f}" if age is not None else "-")
            if age is None:
                problems.append((DiagnosticLevel.WARN, "아직 값이 없다"))
            elif age > data_timeout():
                problems.append((DiagnosticLevel.ERROR, f"{age:.1f}초 동안 새 값이 없다"))
        if new_errors > 0:
            problems.append((DiagnosticLevel.WARN, f"새 실패 {new_errors}번: {hardware.last_error}"))
        if not hardware.is_on:
            problems.append((DiagnosticLevel.WARN, "전원이 꺼져 있다"))
        # 수준은 가장 나쁜 문제를 따르고, 설명에는 문제들을 모두 적는다 (가장 나쁜 것이 맨 앞)
        problems.sort(key=lambda problem: problem[0], reverse=True)
        level = problems[0][0] if problems else DiagnosticLevel.OK
        message = "; ".join(text for _, text in problems) if problems else "ok"
        status.summary(level, message)

    return task
