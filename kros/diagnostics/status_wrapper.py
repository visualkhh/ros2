from kros.msgs.diagnostic_msgs import DiagnosticLevel, DiagnosticStatus, KeyValue


class DiagnosticStatusWrapper:
    """진단 작업(task)이 채우는 상태 빌더 (ROS의 DiagnosticStatusWrapper). summary로 수준과 한 줄 설명을, add로 자세한 값을 적는다.
    summary를 안 부르면 OK, 설명 없음이다."""

    def __init__(self) -> None:
        self.level = DiagnosticLevel.OK
        self.message = ""
        self.values: list[KeyValue] = []

    def summary(self, level: DiagnosticLevel, message: str) -> None:
        self.level, self.message = level, message

    def add(self, key: str, value: object) -> None:
        self.values.append(KeyValue(key, str(value)))

    def build(self, name: str, hardware_id: str = "") -> DiagnosticStatus:
        return DiagnosticStatus(self.level, name, self.message, hardware_id, tuple(self.values))
