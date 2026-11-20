import threading
from collections.abc import Callable, Mapping
from typing import TYPE_CHECKING, Generic, TypeVar, cast

if TYPE_CHECKING:
    from .parameter_event import ParameterEvent

Scalar = bool | int | float | str
ParameterValue = Scalar | tuple[Scalar, ...]
P = TypeVar("P", bound=ParameterValue)

Validator = Callable[[P], bool]  # 새 값이 괜찮으면 True, 거절하면 False
OnChange = Callable[[P], None]  # 값이 바뀐 뒤에 불린다


class ParameterNotDeclaredError(LookupError):
    """선언하지 않은 파라미터를 읽거나 바꾸려 했다."""


class ParameterAlreadyDeclaredError(ValueError):
    """같은 노드에서 같은 이름을 두 번 선언했다."""


class ParameterRejectedError(ValueError):
    """검증(validator)이 새 값을 거절했다."""


def _kind(value: ParameterValue) -> type:
    """파라미터의 타입: 값의 정확한 타입 (bool은 int가 아니다)."""
    if isinstance(value, tuple):
        for item in value:
            if type(item) not in (bool, int, float, str):
                raise TypeError(f"파라미터 값은 bool/int/float/str 또는 그 튜플이어야 한다: {item!r}")
        return tuple
    if type(value) not in (bool, int, float, str):
        raise TypeError(f"파라미터 값은 bool/int/float/str 또는 그 튜플이어야 한다: {value!r}")
    return type(value)


def _same_type(declared: ParameterValue, value: ParameterValue) -> bool:
    if _kind(declared) is not _kind(value):
        return False
    if isinstance(declared, tuple) and isinstance(value, tuple) and declared:
        return all(type(item) is type(declared[0]) for item in value)  # 튜플은 원소 타입이 선언한 것과 같아야 한다
    return True


class _Entry:
    def __init__(self, value: ParameterValue, validator: Validator[ParameterValue] | None, on_change: OnChange[ParameterValue] | None) -> None:
        self.value = value
        self.validator = validator
        self.on_change = on_change


class Parameter(Generic[P]):
    """선언한 파라미터 하나의 손잡이. value는 항상 현재 값이다 (바뀌면 따라 바뀐다)."""

    def __init__(self, manager: "ParameterManager", node: str, name: str) -> None:
        self._manager, self.node, self.name = manager, node, name

    @property
    def value(self) -> P:
        return cast(P, self._manager.get(self.node, self.name))

    def set(self, value: P) -> None:
        self._manager.set(self.node, self.name, value)

    def __repr__(self) -> str:
        return f"Parameter({self.node}.{self.name}={self.value!r})"


class ParameterManager:
    """노드별 파라미터 저장소 (ROS2식: 노드가 선언하고 소유한다). 값의 우선순위는 아래가 낮고 위가 높다:
    선언한 기본값 < Node(parameters=...) < Kros(parameters=...) < 실행 중 set.
    타입은 기본값의 타입으로 고정된다 (다른 타입으로 set하면 TypeError)."""

    def __init__(
        self, overrides: Mapping[str, Mapping[str, ParameterValue]] | None = None, on_event: "Callable[[ParameterEvent], None] | None" = None
    ) -> None:
        self._on_event = on_event  # 선언/변경이 일어나면 불린다 (Context가 parameter_events 토픽에 연결한다). 토픽을 몰라도 동작한다
        self._overrides = {node: dict(values) for node, values in (overrides or {}).items()}  # Kros가 준 값
        self._node_overrides: dict[str, dict[str, ParameterValue]] = {}  # Node 생성자가 준 값
        self._entries: dict[tuple[str, str], _Entry] = {}
        self._lock = threading.RLock()

    def set_node_overrides(self, node: str, values: Mapping[str, ParameterValue]) -> None:
        with self._lock:
            self._node_overrides.setdefault(node, {}).update(values)

    def declare(
        self, node: str, name: str, default: P, validator: Validator[P] | None = None, on_change: OnChange[P] | None = None
    ) -> Parameter[P]:
        """파라미터를 선언한다. 시작 값은 Kros가 준 값, 없으면 Node가 준 값, 없으면 default. 값이 타입/검증에 안 맞으면 선언이 실패한다."""
        _kind(default)
        with self._lock:
            if (node, name) in self._entries:
                raise ParameterAlreadyDeclaredError(f"{node}.{name}는 이미 선언됐다")
            value: ParameterValue = default
            for source in (self._node_overrides, self._overrides):  # 뒤가 이긴다
                if name in source.get(node, {}):
                    value = source[node][name]
            entry = _Entry(value, cast("Validator[ParameterValue] | None", validator), cast("OnChange[ParameterValue] | None", on_change))
            self._check(node, name, default, value, entry.validator)
            self._entries[(node, name)] = _Entry(value, entry.validator, entry.on_change)
            # 기본값(default)이 검증을 못 통과해도 알아챈다 (선언이 잘못된 것)
            if value is not default and entry.validator is not None and not entry.validator(default):
                del self._entries[(node, name)]
                raise ParameterRejectedError(f"{node}.{name}의 기본값 {default!r}가 검증을 통과하지 못한다")
        self._emit(node, name, None, value)
        return Parameter(self, node, name)

    def get(self, node: str, name: str) -> ParameterValue:
        with self._lock:
            return self._entry(node, name).value

    def set(self, node: str, name: str, value: ParameterValue) -> None:
        """값을 바꾼다. 타입이 다르면 TypeError, 검증이 거절하면 ParameterRejectedError(값은 그대로)."""
        with self._lock:
            entry = self._entry(node, name)
            self._check(node, name, entry.value, value, entry.validator)
            old = entry.value
            entry.value = value
            on_change = entry.on_change
        if on_change is not None:
            on_change(value)
        if value != old:  # 같은 값으로 set하면 알리지 않는다
            self._emit(node, name, old, value)

    def _emit(self, node: str, name: str, old: ParameterValue | None, new: ParameterValue) -> None:
        if self._on_event is None:
            return
        from .parameter_event import ParameterEvent, ParameterEventKind  # 순환 import를 피한다

        kind = ParameterEventKind.DECLARED if old is None else ParameterEventKind.CHANGED
        self._on_event(ParameterEvent(node, name, kind, old, new))

    def declared(self, node: str) -> dict[str, ParameterValue]:
        """노드가 선언한 파라미터와 현재 값."""
        with self._lock:
            return {name: entry.value for (n, name), entry in self._entries.items() if n == node}

    def unused_overrides(self) -> list[str]:
        """누군가 값을 줬는데 아무도 선언하지 않은 파라미터 ("노드.이름"). 오타를 찾는 데 쓴다."""
        with self._lock:
            names = {f"{node}.{name}" for source in (self._overrides, self._node_overrides) for node, values in source.items() for name in values}
            declared = {f"{node}.{name}" for node, name in self._entries}
            return sorted(names - declared)

    def _entry(self, node: str, name: str) -> _Entry:
        try:
            return self._entries[(node, name)]
        except KeyError:
            raise ParameterNotDeclaredError(f"{node}.{name}는 선언되지 않았다") from None

    @staticmethod
    def _check(node: str, name: str, declared: ParameterValue, value: ParameterValue, validator: Validator[ParameterValue] | None) -> None:
        if not _same_type(declared, value):
            raise TypeError(f"{node}.{name}는 {_kind(declared).__name__} 타입으로 선언됐다: {value!r} ({_kind(value).__name__})")
        if validator is not None and not validator(value):
            raise ParameterRejectedError(f"{node}.{name}가 값 {value!r}를 거절했다")
