from __future__ import annotations

import random
from abc import ABC, abstractmethod
from collections.abc import Callable
from typing import Generic, TypeVar

from kros.worlds.clock import Clock

T = TypeVar("T")
O = TypeVar("O")


class HardwareError(Exception):
    """하드웨어가 값을 못 읽었거나 명령을 못 실행했다 (통신 끊김, 타임아웃, 체크섬 오류 ...). 하위 클래스의 훅이 일으키면 장치가 기록하고 넘어간다."""


class Hardware(ABC):
    """물리 장치의 최상위 (모터, 센서, 로봇 ...). kros와 무관하고, 월드도 모른다 (시계 신호(Clock)만 받는다: 월드가 내려주든 진짜 하드웨어를 돌리는 실시간 루프가 내려주든 같다).

    - 부품(Hardware)을 달 수 있다. 부품은 달린 장치를 owner로 안다.
    - **period**(초): 이 장치가 일하는 주기. None이면 월드의 시계 신호마다. 장치마다 따로 설정한다.
    - 시계 신호(tick)를 받으면 주기가 됐을 때 **on_period 훅**을 부르고, 부품에게도 내려준다.
      훅 안에서 데이터를 시뮬레이션으로 만들든, 더미로 보내든, 진짜 장치에서 읽든 그 하위 클래스가 정한다.
    - **전원**: power_on()/power_off(). 꺼지면(부모가 꺼져도) 일하지 않는다. 진짜 하드웨어의 전원처럼 명령이다.
    - **실패**: 진짜 하드웨어는 "고장 상태"를 알려 주지 않는다. 그래서 상태 머신은 없고, 사실만 남긴다: 훅이 HardwareError를 내면
      error_count와 last_error에 기록하고 새 데이터를 만들지 않는다. 데이터가 오래됐는지, 고장인지는 쓰는 쪽이 판단한다.
    """

    period: float | None = None  # 하위 클래스의 기본 주기 (생성자 인자로 덮어쓸 수 있다)
    name: str = ""  # 부품 이름 (달 때 정한다: "left_wheel", "lidar" ...). 진단 이름 등에 쓴다
    model: str = ""  # 모델 번호 ("LDS-01", "XL430-W250" ...). 진짜 장치의 식별자(hardware_id)가 된다

    def __init__(self, period: float | None = None) -> None:
        if period is not None:
            self.period = period
        self.parts: list[Hardware] = []
        self.owner: Hardware | None = None  # 이 장치가 달려 있는 장치 (없으면 최상위)
        self.powered = True  # 이 장치의 전원 스위치 (부모가 꺼져 있으면 is_on은 False)
        self.error_count = 0  # 지금까지 실패한 횟수
        self.last_error: HardwareError | None = None  # 가장 최근 실패
        self._next_due = 0.0  # 다음 주기 시각. 간격을 빼서 비교하면 부동소수점 오차로 한 번씩 밀린다
        self._last_run: float | None = None

    def add(self, part: Hardware, name: str | None = None) -> Hardware:
        """부품을 단다 (name을 주면 부품의 이름으로 정한다). 단 부품을 그대로 돌려준다."""
        if name is not None:
            part.name = name
        part.owner = self
        self.parts.append(part)
        return part

    def find_owner(self, kind: type[O]) -> O | None:
        """이 장치가 달려 있는 장치들을 위로 올라가며 처음 만나는 kind (예: 부품이 보드에 달리고 보드가 로봇에 달려 있어도 로봇을 찾는다). 없으면 None."""
        node: Hardware | None = self.owner
        while node is not None and not isinstance(node, kind):
            node = node.owner
        return node

    def power_on(self) -> None:
        self.powered = True

    def power_off(self) -> None:
        self.powered = False

    @property
    def is_on(self) -> bool:
        """실제로 전원이 들어와 있는가: 자기 스위치와 달려 있는 장치의 전원이 모두 켜져 있어야 한다."""
        return self.powered and (self.owner is None or self.owner.is_on)

    def tick(self, clock: Clock) -> None:
        """시계 신호를 받는다: 켜져 있고 주기가 됐으면 on_period 훅을 부르고, 부품들에게도 그대로 내려준다."""
        if self.is_on:
            due = self._due(clock)
            if due is not None:
                self.on_period(due)
        else:
            self._last_run = None  # 다시 켜졌을 때 꺼져 있던 시간이 한 번에 흘러온 것처럼 보이지 않게
        for part in self.parts:
            part.tick(clock)

    def fail(self, error: HardwareError) -> None:
        """실패를 기록한다 (사실만 남긴다)."""
        self.error_count += 1
        self.last_error = error

    def on_period(self, clock: Clock) -> None:
        """훅: 이 장치의 주기마다 불린다. clock.dt는 지난 호출 이후 지난 시간. 하위 클래스가 덮어쓴다."""

    def _due(self, clock: Clock) -> Clock | None:
        if self.period is None:
            return clock  # 매 신호마다
        if clock.time + 1e-9 < self._next_due:
            return None
        while self._next_due <= clock.time + 1e-9:  # dt가 커서 여러 주기를 건너뛰어도 한 번만
            self._next_due += self.period
        elapsed = clock.dt if self._last_run is None else clock.time - self._last_run
        self._last_run = clock.time
        return Clock(clock.time, elapsed)


class Sensor(Hardware, Generic[T]):
    """데이터를 내보내는 장치. 받는 방법 두 가지: read()(최신값), subscribe(콜백)(새 값이 나올 때마다).

    새 값을 만드는 곳은 sample() 훅 하나다: 시뮬레이션 구현은 월드를 보고 만들고, 더미는 고정값을, 진짜 구현은 장치에서 읽는다.
    """

    def __init__(self, period: float | None = None) -> None:
        super().__init__(period)
        self.latest: T | None = None
        self.stamp: float | None = None  # 최신 값이 만들어진 월드 시각. 쓰는 쪽이 (지금 - stamp)로 "오래됐다"를 판단한다
        self.fault: Callable[[Clock], bool] | None = None  # 시뮬레이션/시험용 고장 주입: True를 돌려주면 이번 읽기는 실패한다
        self.latency = 0.0  # 초: 측정하고 나서 값이 나오기까지 걸리는 시간 (통신/처리 지연). 값에 찍힌 시각(stamp)은 측정한 때라 받는 쪽에는 그만큼 오래된 값이다
        self.jitter = 0.0  # 초: 지연이 매번 흔들리는 정도 (표준편차). 지연은 0 아래로 내려가지 않고 값의 순서는 바뀌지 않는다
        self.delay_seed: int | None = None
        self._delay_rng: random.Random | None = None
        self._in_flight: list[tuple[float, T, float]] = []  # (나올 시각, 값, 측정 시각)
        self._subscribers: list[Callable[[T], None]] = []

    def subscribe(self, callback: Callable[[T], None]) -> None:
        """새 값이 나올 때마다 callback(값)이 불린다."""
        self._subscribers.append(callback)

    def read(self) -> T | None:
        """가장 최근 값 (아직 없으면 None)."""
        return self.latest

    def on_period(self, clock: Clock) -> None:
        try:
            if self.fault is not None and self.fault(clock):
                raise HardwareError("주입된 고장")
            data = self.sample(clock)
        except HardwareError as error:  # 읽기 실패: 기록만 하고 새 값은 없다 (latest는 그대로 남아 오래된 값이 된다)
            self.fail(error)
            return
        if data is None:
            return
        if self.latency == 0.0 and self.jitter == 0.0 and not self._in_flight:
            self._deliver(data, clock.time)
            return
        if self._delay_rng is None:
            self._delay_rng = random.Random(self.delay_seed)
        ready = clock.time + max(0.0, self.latency + self._delay_rng.gauss(0.0, self.jitter))
        if self._in_flight:
            ready = max(ready, self._in_flight[-1][0])  # 먼저 측정한 값이 먼저 나온다
        self._in_flight.append((ready, data, clock.time))

    def _deliver(self, data: T, stamp: float) -> None:
        self.latest = data
        self.stamp = stamp
        for callback in self._subscribers:
            callback(data)

    def tick(self, clock: Clock) -> None:
        super().tick(clock)
        if not self.is_on:
            self._in_flight.clear()  # 꺼지면 오는 중이던 값도 사라진다
        while self._in_flight and self._in_flight[0][0] <= clock.time + 1e-9:
            _, data, stamp = self._in_flight.pop(0)
            self._deliver(data, stamp)

    @abstractmethod
    def sample(self, clock: Clock) -> T | None:
        """훅: 새 값을 만든다. 없으면 None. 읽기에 실패하면 HardwareError를 일으킨다."""


class Actuator(Hardware, Generic[T]):
    """명령을 받는 장치. write(명령)으로 명령을 내리면 다음 주기에 apply() 훅이 그 명령을 실행한다.

    apply()에서 시뮬레이션 구현은 월드 안에서 움직이고, 진짜 구현은 장치로 명령을 보낸다.
    """

    def __init__(self, idle: T, period: float | None = None) -> None:
        super().__init__(period)
        self.command: T = idle  # 가장 최근에 받은 명령

    def write(self, command: T) -> None:
        """명령을 내린다 (객체째 교체라 다른 스레드에서 불러도 안전하다)."""
        self.command = command

    def on_period(self, clock: Clock) -> None:
        try:
            self.apply(self.command, clock)
        except HardwareError as error:  # 명령 실행 실패: 기록만 한다
            self.fail(error)

    @abstractmethod
    def apply(self, command: T, clock: Clock) -> None:
        """훅: 명령을 실행한다. 실패하면 HardwareError를 일으킨다."""
