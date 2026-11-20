from enum import Enum


class GoalStatus(Enum):
    ACCEPTED = "accepted"  # 서버가 받았고 아직 결과 없음
    SUCCEEDED = "succeeded"
    CANCELED = "canceled"
    ABORTED = "aborted"  # execute가 예외를 던지거나 잘못된 결과 타입
    REJECTED = "rejected"  # 서버가 goal을 거절
