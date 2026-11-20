class TransformException(Exception):
    """TF 조회가 실패했다 (아래 세 가지의 부모)."""


class LookupException(TransformException):
    """그런 좌표계를 모른다 (아직 아무도 그 좌표계의 변환을 내보내지 않았다)."""


class ConnectivityException(TransformException):
    """두 좌표계 모두 알지만 변환 트리에서 이어져 있지 않다."""


class ExtrapolationException(TransformException):
    """요청한 시각이 보관 중인 이력 범위 밖이다 (미래이거나 너무 오래됐다)."""
