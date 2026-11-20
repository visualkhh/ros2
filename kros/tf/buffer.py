import bisect
import logging
import threading
import time as _time

from kros.msgs.geometry_msgs import Point, PointStamped, Pose, PoseStamped, Transform, TransformStamped, Vector3
from kros.msgs.std_msgs import Header

from .errors import ConnectivityException, ExtrapolationException, LookupException, TransformException
from .transform_math import IDENTITY, apply, compose, interpolate, inverse, quaternion_multiply, quaternion_normalize

logger = logging.getLogger(__name__)
EPSILON = 1e-9  # 시각 비교의 부동소수점 오차 허용


class Buffer:
    """좌표계 트리와 변환 이력 보관소 (ROS의 tf2_ros.Buffer). 순수 계산이고 kros의 다른 부분을 모른다.

    - 좌표계는 트리다: 자식은 부모가 하나. set_transform(parent, child, ...)이 그 변이다.
    - 움직이는 변환은 시각별 이력을 cache_time(초) 동안 보관하고, 조회할 때 앞뒤 두 값 사이를 보간한다
      (이동은 선형, 회전은 구면 보간). 안 변하는(static) 변환은 하나만 들고 있고 시각과 상관없다.
    - lookup_transform(target, source, time): source 좌표계의 점을 target 좌표계로 옮기는 변환.
      time이 None이면 모든 움직이는 변환이 가진 가장 최근 공통 시각이다 (ROS의 Time(0)).
    - timeout(초)을 주면 변환이 도착할 때까지 그만큼 기다린다. 기다리는 동안 set_transform이 깨워 준다.
      주의: TF 콜백(구독 스레드) 안에서 기다리면 그 변환을 채울 스레드를 막으니 쓰지 않는다.
    """

    def __init__(self, cache_time: float = 10.0) -> None:
        if cache_time <= 0:
            raise ValueError(f"cache_time은 0보다 커야 한다: {cache_time}")
        self.cache_time = cache_time
        self._lock = threading.RLock()
        self._arrived = threading.Condition(self._lock)  # 변환이 들어올 때마다 알린다
        self._parent: dict[str, str] = {}  # 자식 -> 부모
        self._static: dict[str, Transform] = {}  # 자식 -> 안 변하는 변환
        self._stamps: dict[str, list[float]] = {}  # 자식 -> 이력의 시각들 (정렬됨)
        self._samples: dict[str, list[Transform]] = {}  # 자식 -> 이력의 변환들 (_stamps와 같은 순서)

    def set_transform(self, parent: str, child: str, stamp: float, transform: Transform, is_static: bool = False) -> None:
        """parent 좌표계에서 본 child 좌표계의 변환을 넣는다. child의 부모가 바뀌면 이전 이력을 버리고(경고), 순환이 생기면 ConnectivityException."""
        with self._arrived:
            self._set(parent, child, stamp, transform, is_static)
            self._arrived.notify_all()

    def _set(self, parent: str, child: str, stamp: float, transform: Transform, is_static: bool = False) -> None:
        if parent == child:
            raise ValueError(f"좌표계가 자기 자신의 자식일 수 없다: {child!r}")
        transform = Transform(transform.translation, quaternion_normalize(transform.rotation))
        with self._lock:
            if self._parent.get(child, parent) != parent:
                logger.warning("좌표계 %r의 부모가 %r에서 %r로 바뀌었다 (이전 이력은 버린다)", child, self._parent[child], parent)
                self._forget(child)
            ancestor: str | None = parent
            while ancestor is not None:  # parent의 조상 중에 child가 있으면 순환이다
                if ancestor == child:
                    raise ConnectivityException(f"좌표계 순환이 생긴다: {parent!r} -> {child!r}")
                ancestor = self._parent.get(ancestor)
            self._parent[child] = parent
            if is_static:
                self._clear_history(child)
                self._static[child] = transform
                return
            self._static.pop(child, None)  # 움직이는 변환이 오면 정적 변환은 대체된다
            stamps, samples = self._stamps.setdefault(child, []), self._samples.setdefault(child, [])
            latest = stamps[-1] if stamps else stamp
            if stamp < latest - self.cache_time:
                return  # 보관 범위보다 오래된 데이터는 무시
            index = bisect.bisect_left(stamps, stamp)
            if index < len(stamps) and abs(stamps[index] - stamp) <= EPSILON:
                samples[index] = transform  # 같은 시각이면 덮어쓴다
            else:
                stamps.insert(index, stamp)
                samples.insert(index, transform)
            cutoff = stamps[-1] - self.cache_time
            drop = bisect.bisect_left(stamps, cutoff - EPSILON)
            if drop:
                del stamps[:drop], samples[:drop]

    def lookup_transform(self, target: str, source: str, time: float | None = None, timeout: float = 0.0) -> TransformStamped:
        """source 좌표계의 점을 target 좌표계로 옮기는 변환. 결과의 header.frame_id는 target, child_frame_id는 source다.

        LookupException: 모르는 좌표계. ConnectivityException: 이어져 있지 않다. ExtrapolationException: time이 이력 범위 밖.
        timeout 안에 조회가 되면 그 값을, 끝내 안 되면 마지막 예외를 던진다.
        """
        deadline = _time.monotonic() + timeout
        with self._arrived:
            while True:
                try:
                    return self._lookup(target, source, time)
                except TransformException:
                    remaining = deadline - _time.monotonic()
                    if remaining <= 0:
                        raise
                    self._arrived.wait(remaining)

    def _lookup(self, target: str, source: str, time: float | None) -> TransformStamped:
        with self._lock:
            for frame in (target, source):
                if not self._known(frame):
                    raise LookupException(f"좌표계 {frame!r}를 모른다 (아직 그 좌표계의 변환을 내보낸 곳이 없다)")
            target_chain, source_chain = self._chain(target), self._chain(source)
            common = next((f for f in source_chain if f in set(target_chain)), None)
            if common is None:
                raise ConnectivityException(f"좌표계 {target!r}와 {source!r}는 변환 트리에서 이어져 있지 않다")
            source_edges = source_chain[: source_chain.index(common)]  # common에 닿기까지 지나는 변들 (자식 이름으로 가리킨다)
            target_edges = target_chain[: target_chain.index(common)]
            when = self._latest_common_time(source_edges + target_edges) if time is None else time
            to_common_from_source = self._fold(source_edges, when)
            to_common_from_target = self._fold(target_edges, when)
            result = compose(inverse(to_common_from_target), to_common_from_source)
            return TransformStamped(Header(when, target), source, result)

    def transform_point(self, target: str, source: str, point: Vector3, time: float | None = None, timeout: float = 0.0) -> Vector3:
        """source 좌표계에서 본 점을 target 좌표계에서 본 점으로 바꾼다 (lookup_transform + 적용). 조회가 실패하면 lookup_transform과 같은 예외."""
        return apply(self.lookup_transform(target, source, time, timeout).transform, point)

    def transform(self, stamped: PointStamped | PoseStamped, target_frame: str, timeout: float = 0.0) -> PointStamped | PoseStamped:
        """시각과 좌표계가 붙은 점/자세를 target_frame으로 옮긴다 (ROS의 buffer.transform).
        header.stamp 시각의 변환을 쓰고, ROS의 Time(0)처럼 stamp가 0이면 가장 최근 값을 쓴다."""
        stamp = stamped.header.stamp
        t = self.lookup_transform(target_frame, stamped.header.frame_id, stamp or None, timeout)
        header = Header(t.header.stamp, target_frame)
        if isinstance(stamped, PointStamped):
            p = apply(t.transform, Vector3(stamped.point.x, stamped.point.y, stamped.point.z))
            return PointStamped(header, Point(p.x, p.y, p.z))
        pose = stamped.pose
        p = apply(t.transform, Vector3(pose.position.x, pose.position.y, pose.position.z))
        q = quaternion_normalize(quaternion_multiply(t.transform.rotation, pose.orientation))
        return PoseStamped(header, Pose(Point(p.x, p.y, p.z), q))

    def can_transform(self, target: str, source: str, time: float | None = None, timeout: float = 0.0) -> bool:
        try:
            self.lookup_transform(target, source, time, timeout)
        except TransformException:
            return False
        return True

    def all_frames_as_string(self) -> str:
        """좌표계들의 부모를 한 줄씩 (ROS와 같은 모양: "Frame X exists with parent Y.")."""
        with self._lock:
            return "".join(f"Frame {frame} exists with parent {self._parent.get(frame, 'NO_PARENT')}.\n" for frame in self.all_frames())

    def all_frames_as_yaml(self) -> str:
        """좌표계별 부모와 이력 정보를 YAML 문자열로 (ROS의 all_frames_as_yaml). 방송 주기는 이력에서 구한다."""
        with self._lock:
            lines: list[str] = []
            for child in sorted(self._parent):
                stamps = self._stamps.get(child, [])
                span = stamps[-1] - stamps[0] if len(stamps) > 1 else 0.0
                rate = (len(stamps) - 1) / span if span > 0 else 0.0
                lines += [
                    f"{child}:",
                    f"  parent: '{self._parent[child]}'",
                    f"  broadcaster: '{'static' if child in self._static else 'tf'}'",
                    f"  rate: {rate:.3f}",
                    f"  most_recent_transform: {stamps[-1] if stamps else 0.0:.6f}",
                    f"  oldest_transform: {stamps[0] if stamps else 0.0:.6f}",
                    f"  buffer_length: {span:.3f}",
                ]
            return "\n".join(lines) + ("\n" if lines else "")

    def parent_of(self, frame: str) -> str | None:
        """frame의 부모 좌표계. 뿌리이거나 모르는 좌표계면 None."""
        with self._lock:
            return self._parent.get(frame)

    def all_frames(self) -> list[str]:
        with self._lock:
            return sorted(set(self._parent) | set(self._parent.values()))

    def clear(self) -> None:
        with self._lock:
            self._parent.clear()
            self._static.clear()
            self._stamps.clear()
            self._samples.clear()

    # ---- 내부 -------------------------------------------------------------------------------

    def _known(self, frame: str) -> bool:
        return frame in self._parent or frame in self._parent.values()

    def _chain(self, frame: str) -> list[str]:
        """frame에서 뿌리까지 (frame 포함)."""
        chain = [frame]
        while chain[-1] in self._parent:
            chain.append(self._parent[chain[-1]])
        return chain

    def _latest_common_time(self, children: list[str]) -> float:
        lasts = [self._stamps[child][-1] for child in children if child not in self._static and self._stamps.get(child)]
        return min(lasts) if lasts else 0.0  # 움직이는 변환이 하나도 없으면 시각은 의미가 없다

    def _fold(self, children: list[str], time: float) -> Transform:
        """children(아래에서 위로)의 변들을 이어서 맨 위 조상 좌표계에서 본 맨 아래 좌표계의 변환을 만든다."""
        acc = IDENTITY
        for child in children:
            acc = compose(self._edge(child, time), acc)
        return acc

    def _edge(self, child: str, time: float) -> Transform:
        if child in self._static:
            return self._static[child]
        stamps, samples = self._stamps.get(child, []), self._samples.get(child, [])
        if not stamps:
            raise ConnectivityException(f"좌표계 {child!r}의 변환 값이 아직 없다")
        if time < stamps[0] - EPSILON or time > stamps[-1] + EPSILON:
            raise ExtrapolationException(f"좌표계 {child!r}의 이력은 {stamps[0]:.3f}~{stamps[-1]:.3f}초인데 {time:.3f}초를 요청했다")
        index = bisect.bisect_left(stamps, time - EPSILON)
        if index == 0 or abs(stamps[index] - time) <= EPSILON:
            return samples[min(index, len(samples) - 1)]
        before, after = index - 1, index
        ratio = (time - stamps[before]) / (stamps[after] - stamps[before])
        return interpolate(samples[before], samples[after], ratio)

    def _clear_history(self, child: str) -> None:
        self._stamps.pop(child, None)
        self._samples.pop(child, None)

    def _forget(self, child: str) -> None:
        self._static.pop(child, None)
        self._clear_history(child)
