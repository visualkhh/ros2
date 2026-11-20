"""액션이 내부적으로 쓰는 서비스/토픽. ROS처럼 액션 = 서비스 2개 + 토픽 2개 위에 올라간다."""
from kros.service import ServiceType

from .cancel_goal_request import CancelGoalRequest
from .cancel_goal_response import CancelGoalResponse
from .send_goal_request import SendGoalRequest
from .send_goal_response import SendGoalResponse

SEND_GOAL = ServiceType("SendGoal", SendGoalRequest, SendGoalResponse)
CANCEL_GOAL = ServiceType("CancelGoal", CancelGoalRequest, CancelGoalResponse)


def send_goal_service(name: str) -> str:
    return f"{name}/send_goal"


def cancel_goal_service(name: str) -> str:
    return f"{name}/cancel_goal"


def goal_topic(name: str) -> str:  # 서버 내부용: 받아들인 goal을 실행 스레드로 넘기는 큐
    return f"{name}/_goal"


def event_topic(name: str) -> str:  # feedback과 result를 같은 순서 큐로
    return f"{name}/event"
