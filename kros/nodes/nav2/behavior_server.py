"""nav2 behavior_server 흉내: spin · backup · wait 회복 동작.

Nav2의 behavior_server(nav2_behaviors)가 제공하는 회복 행동을 단순화한 것이다. cmd_vel 토픽에 직접 속도를 내고,
odom으로 진행을 확인한다. bt_navigator가 controller 실패 시 이 행동들을 불러 회복을 시도한다.
"""
import math
import time
from collections.abc import Mapping

from kros.action.server_goal_handle import ServerGoalHandle
from kros.context import Context
from kros.msgs.geometry_msgs import Twist, Vector3
from kros.msgs.nav2_msgs import BackUp, BackUpFeedback, BackUpGoal, BackUpResult, Spin, SpinFeedback, SpinGoal, SpinResult, Wait, WaitFeedback, WaitGoal, WaitResult
from kros.msgs.nav_msgs import Odometry
from kros.msgs.std_msgs import Header
from kros.nodes.node import Node
from kros.parameter import ParameterValue

CMD_VEL_TOPIC_NAME = "cmd_vel"


class BehaviorServer(Node):
    """회복 동작 서버: spin(제자리 회전) · backup(후진) · wait(대기)."""

    def __init__(self, context: Context, name: str | None = None, parameters: Mapping[str, ParameterValue] | None = None) -> None:
        super().__init__(context, name or "behavior_server", parameters)
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", CMD_VEL_TOPIC_NAME)
        self.odom_topic = self.declare_parameter("odom_topic", "odom")
        self.max_ang = self.declare_parameter("max_ang", 1.0, validator=lambda v: v > 0)
        self.max_lin = self.declare_parameter("max_lin", 0.1, validator=lambda v: v > 0)
        self._odom_yaw: float | None = None
        self._odom_x: float | None = None
        self._odom_y: float | None = None
        self.subscribe(self.odom_topic.value, Odometry, self._on_odom)
        self.create_action_server("spin", Spin, self._execute_spin)
        self.create_action_server("backup", BackUp, self._execute_backup)
        self.create_action_server("wait", Wait, self._execute_wait)

    def _on_odom(self, odom: Odometry) -> None:
        p = odom.pose.pose.position
        self._odom_x, self._odom_y = p.x, p.y
        q = odom.pose.pose.orientation
        self._odom_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def _publish(self, v: float, w: float) -> None:
        self.publish(self.cmd_vel_topic.value, Twist(linear=Vector3(x=v), angular=Vector3(z=w)))

    def _stop(self) -> None:
        self._publish(0.0, 0.0)

    def _execute_spin(self, handle: ServerGoalHandle[SpinGoal, SpinFeedback]) -> SpinResult:
        goal = handle.goal
        R = SpinResult
        target = abs(goal.target_yaw)
        started = self.now()
        traveled = 0.0
        prev_yaw = self._odom_yaw
        sign = 1.0 if goal.target_yaw >= 0 else -1.0
        while True:
            if handle.is_cancel_requested:
                self._stop()
                return R(R.NONE, "취소")
            if self.now() - started >= goal.time_allowance:
                self._stop()
                return R(R.TIMEOUT, f"{goal.time_allowance}초 안에 회전하지 못했다")
            if traveled >= target:
                self._stop()
                return R(R.NONE)
            self._publish(0.0, sign * self.max_ang.value)
            if prev_yaw is not None and self._odom_yaw is not None:
                traveled += abs(math.atan2(math.sin(self._odom_yaw - prev_yaw), math.cos(self._odom_yaw - prev_yaw)))
                prev_yaw = self._odom_yaw
            handle.publish_feedback(SpinFeedback(traveled, self.now() - started))
            time.sleep(0.05)
        self._stop()
        return R(R.NONE)

    def _execute_backup(self, handle: ServerGoalHandle[BackUpGoal, BackUpFeedback]) -> BackUpResult:
        goal = handle.goal
        R = BackUpResult
        target = abs(goal.backup_dist)
        started = self.now()
        traveled = 0.0
        prev = (self._odom_x, self._odom_y) if self._odom_x is not None else None
        while True:
            if handle.is_cancel_requested:
                self._stop()
                return R(R.NONE, "취소")
            if self.now() - started >= goal.time_allowance:
                self._stop()
                return R(R.TIMEOUT, f"{goal.time_allowance}초 안에 후진하지 못했다")
            if traveled >= target:
                self._stop()
                return R(R.NONE)
            self._publish(-abs(goal.backup_speed), 0.0)
            if prev is not None and self._odom_x is not None and self._odom_y is not None:
                px, py = prev
                if px is not None and py is not None:
                    traveled += math.hypot(self._odom_x - px, self._odom_y - py)
                    prev = (self._odom_x, self._odom_y)
            handle.publish_feedback(BackUpFeedback(traveled, self.now() - started))
            time.sleep(0.05)
        self._stop()
        return R(R.NONE)

    def _execute_wait(self, handle: ServerGoalHandle[WaitGoal, WaitFeedback]) -> WaitResult:
        goal = handle.goal
        R = WaitResult
        started = self.now()
        duration = goal.wait_duration
        while self.now() - started < duration:
            if handle.is_cancel_requested:
                self._stop()
                return R(R.NONE, "취소")
            handle.publish_feedback(WaitFeedback(max(0.0, duration - (self.now() - started))))
            time.sleep(0.05)
        self._stop()
        return R(R.NONE)