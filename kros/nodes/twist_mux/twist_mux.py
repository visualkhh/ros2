import threading
from collections.abc import Mapping

from kros.context import Context
from kros.msgs.geometry_msgs import Twist
from kros.msgs.std_msgs import Bool
from kros.nodes.node import Node
from kros.parameter import Parameter, ParameterValue


class TwistMux(Node):
    """ROS twist_mux 흉내: 여러 속도 명령 토픽 중 우선순위가 가장 높고 살아 있는(timeout 안에 받은) 것 하나만 cmd_vel_out으로 내보낸다.
    lock(std_msgs/Bool)이 켜져 있으면 그 priority 이하의 입력은 막힌다 (비상정지 버튼 등). lock에 timeout을 주면 그 안에 신호가 안 와도 켜진 것으로 본다.

    파라미터 (twist_mux의 topics/locks yaml을 kros 파라미터로 펼친 것):
      topics: 입력 이름들, "<이름>.topic", "<이름>.timeout" (초), "<이름>.priority" (0~255, 클수록 이긴다)
      locks: lock 이름들, "<이름>.topic", "<이름>.timeout" (0이면 시간으로는 안 잠근다), "<이름>.priority"
      cmd_vel_out: 내보낼 토픽 (twist_mux 기본 "cmd_vel_out", 보통 remap으로 바꾼다)
    가장 높은 입력이 timeout으로 끊기면 아무것도 내보내지 않는다 (twist_mux와 같다: 멈추는 것은 받는 쪽/다음 단계 몫).
    """

    def __init__(self, context: Context, name: str | None = None, parameters: Mapping[str, ParameterValue] | None = None) -> None:
        super().__init__(context, name or "twist_mux", parameters)
        positive = lambda value: value > 0  # noqa: E731
        priority = lambda value: 0 <= value <= 255  # noqa: E731
        self.cmd_vel_out = self.declare_parameter("cmd_vel_out", "cmd_vel_out")
        self.topics = self.declare_parameter("topics", ("navigation", "teleop"))
        self.locks: Parameter[tuple[str, ...]] = self.declare_parameter("locks", ())
        self._lock = threading.Lock()
        self._inputs: dict[str, tuple[float, int]] = {}  # 이름 -> (timeout, priority)
        self._last: dict[str, float] = {}  # 이름 -> 마지막으로 받은 시각
        defaults = {"navigation": ("cmd_vel_nav", 0.5, 10), "teleop": ("cmd_vel_teleop", 0.5, 100)}
        for name in self.topics.value:
            topic, timeout, level = defaults.get(str(name), (str(name), 0.5, 50))
            topic_p = self.declare_parameter(f"{name}.topic", topic)
            timeout_p = self.declare_parameter(f"{name}.timeout", timeout, validator=positive)
            level_p = self.declare_parameter(f"{name}.priority", level, validator=priority)
            self._inputs[str(name)] = (timeout_p.value, level_p.value)
            self.subscribe(topic_p.value, Twist, lambda twist, key=str(name): self._on_twist(key, twist))  # type: ignore[misc]
        self._locks: dict[str, tuple[float, int]] = {}
        self._lock_state: dict[str, bool] = {}
        for name in self.locks.value:
            topic_p = self.declare_parameter(f"{name}.topic", str(name))
            timeout_p = self.declare_parameter(f"{name}.timeout", 0.0, validator=lambda value: value >= 0)
            level_p = self.declare_parameter(f"{name}.priority", 255, validator=priority)
            self._locks[str(name)] = (timeout_p.value, level_p.value)
            self.subscribe(topic_p.value, Bool, lambda flag, key=str(name): self._on_lock(key, flag))  # type: ignore[misc]
        self._started = self.now()

    def _on_lock(self, name: str, flag: Bool) -> None:
        with self._lock:
            self._lock_state[name] = flag.data
            self._last[f"lock:{name}"] = self.now()

    def lock_priority(self, now: float) -> int:
        """켜진 lock 중 가장 높은 priority (없으면 -1). timeout이 있는 lock은 그 안에 신호가 없으면 켜진 것."""
        level = -1
        for name, (timeout, priority) in self._locks.items():
            last = self._last.get(f"lock:{name}", self._started)
            if self._lock_state.get(name, False) or (timeout > 0 and now - last > timeout):
                level = max(level, priority)
        return level

    def active(self, now: float) -> str | None:
        """지금 이기는 입력: timeout 안에 받았고 lock보다 priority가 높은 것 중 가장 높은 것."""
        blocked = self.lock_priority(now)
        alive = [(priority, name) for name, (timeout, priority) in self._inputs.items() if name in self._last and now - self._last[name] <= timeout and priority > blocked]
        return max(alive)[1] if alive else None

    def _on_twist(self, name: str, twist: Twist) -> None:
        now = self.now()
        with self._lock:
            self._last[name] = now
            winner = self.active(now)
        if winner == name:
            self.publish(self.cmd_vel_out.value, twist)
