import math
import threading
import time
from collections.abc import Mapping

from kros.action import ClientGoalHandle, GoalStatus
from kros.action.server_goal_handle import ServerGoalHandle
from kros.context import Context
from kros.msgs.geometry_msgs import Point, Pose, PoseStamped
from kros.msgs.nav2_msgs import (
    ComputePathToPose, ComputePathToPoseGoal, ComputePathToPoseResult, FollowPath, FollowPathGoal, FollowPathResult,
    NavigateToPose, NavigateToPoseFeedback, NavigateToPoseGoal, NavigateToPoseResult,
)
from kros.msgs.nav_msgs import Path
from kros.msgs.std_msgs import Bool, Header
from kros.nodes.node import Node
from kros.parameter import ParameterValue
from kros.tf import Buffer, TransformException, TransformListener
from kros.topic import QOS_LATCHED

R = NavigateToPoseResult
GOAL_POSE_TOPIC_NAME = "goal_pose"
GOAL_POSES_TOPIC_NAME = "goal_poses"
CANCEL_GOAL_TOPIC_NAME = "cancel_goal"
REPLACED = "새 경로로 바뀌었다"  # controller_server가 갈아탄 옛 경로에 돌려주는 메시지


class BtNavigator(Node):
    """nav2 bt_navigator 흉내: navigate_to_pose 액션(nav2_msgs/NavigateToPose)을 받아 planner_server로 경로를 계획하고 controller_server로 따라가게 한다.
    nav2 기본 BT(navigate_to_pose_w_replanning_and_recovery)의 흐름을 코드로 고정했다: replanning_period마다 다시 계획해 새 경로를 보내고
    (controller가 갈아탄다), controller가 실패하면 number_of_recoveries번까지 다시 계획해 이어 간다. 새 goal이 오면 지금 goal은 물러난다.
    goal_pose 토픽(geometry_msgs/PoseStamped, rviz의 2D Goal Pose)을 받으면 자기에게 navigate_to_pose를 보낸다 (nav2와 같다).

    파라미터: replanning_period (초, 시뮬레이션 시간), number_of_recoveries, global_frame ("map"), robot_base_frame ("base_footprint"), goal_topic ("goal_pose").
    """

    def __init__(self, context: Context, name: str | None = None, parameters: Mapping[str, ParameterValue] | None = None) -> None:
        super().__init__(context, name or "bt_navigator", parameters)
        self.replanning_period = self.declare_parameter("replanning_period", 1.0, validator=lambda value: value > 0)
        self.number_of_recoveries = self.declare_parameter("number_of_recoveries", 3, validator=lambda value: value >= 0)
        self.global_frame = self.declare_parameter("global_frame", "map")
        self.robot_base_frame = self.declare_parameter("robot_base_frame", "base_footprint")
        self.goal_topic = self.declare_parameter("goal_topic", GOAL_POSE_TOPIC_NAME)
        self.goal_poses_topic = self.declare_parameter("goal_poses_topic", GOAL_POSES_TOPIC_NAME)
        self.cancel_topic = self.declare_parameter("cancel_topic", CANCEL_GOAL_TOPIC_NAME)
        self.buffer = Buffer()
        self._listener = TransformListener(self.buffer, self)
        self._planner = self.create_action_client("compute_path_to_pose", ComputePathToPose)
        self._controller = self.create_action_client("follow_path", FollowPath)
        self._self_client = self.create_action_client("navigate_to_pose", NavigateToPose)
        self._preempt = threading.Event()  # 새 goal이 왔다 (액션 서버는 goal을 차례로 돌린다: 지금 것이 물러나야 새 것이 시작된다)
        self._cancel_requested = False
        self._follow_handle: ClientGoalHandle | None = None
        self._through_thread: threading.Thread | None = None
        self.create_action_server("navigate_to_pose", NavigateToPose, self._execute, goal_callback=self._on_new_goal)
        self.subscribe(self.goal_topic.value, PoseStamped, self._on_goal_pose)
        self.subscribe(self.goal_poses_topic.value, Path, self._on_goal_poses)
        self.subscribe(self.cancel_topic.value, Bool, self._on_cancel)

    def _on_goal_pose(self, pose: PoseStamped) -> None:
        self.get_logger().info("goal_pose: (%.2f, %.2f)", pose.pose.position.x, pose.pose.position.y)
        self._cancel_requested = True  # 진행 중인 through-poses가 있으면 멈춘다
        if self._follow_handle is not None:
            try:
                self._follow_handle.cancel(5.0)
            except LookupError:
                pass
        self._cancel_requested = False
        self._self_client.send_goal(NavigateToPoseGoal(pose), timeout=5.0)

    def _on_goal_poses(self, path: Path) -> None:
        if not path.poses:
            return
        self.get_logger().info("goal_poses: %d goals", len(path.poses))
        if self._through_thread is not None and self._through_thread.is_alive():
            self._cancel_requested = True
            self._through_thread.join(timeout=1.0)
        self._cancel_requested = False
        self._through_thread = threading.Thread(target=self._run_through, args=(list(path.poses),), daemon=True)
        self._through_thread.start()

    def _on_cancel(self, _msg: Bool) -> None:
        self.get_logger().info("cancel requested")
        self._cancel_requested = True
        self._preempt.set()
        if self._follow_handle is not None:
            try:
                self._follow_handle.cancel(5.0)
            except LookupError:
                pass
        self.publish("plan", Path(Header(self.now(), self.global_frame.value)), QOS_LATCHED)  # viz 경로 지우기

    def _plan_segment(self, start: PoseStamped, goal: PoseStamped) -> ComputePathToPoseResult:
        message = self._planner.send_goal(ComputePathToPoseGoal(goal=goal, start=start, use_start=True), timeout=5.0)
        result = message.result(30.0).result
        if not isinstance(result, ComputePathToPoseResult):
            return ComputePathToPoseResult(error_code=ComputePathToPoseResult.UNKNOWN, error_msg=f"planner_server 결과가 없다 ({message.status.name})")
        return result

    def _run_through(self, goals: list[PoseStamped]) -> None:
        # 각 구간을 계획해 하나의 경로로 이어 붙인 뒤 controller에 보낸다 (ROS의 navigate_through_poses와 비슷).
        try:
            t = self.buffer.lookup_transform(self.global_frame.value, self.robot_base_frame.value).transform
        except TransformException:
            self.get_logger().warn("goal_poses: 로봇 위치를 모른다")
            return
        current = PoseStamped(Header(self.now(), self.global_frame.value), Pose(Point(t.translation.x, t.translation.y, 0.0), t.rotation))
        combined: list[PoseStamped] = []
        for goal in goals:
            if self._cancel_requested:
                return
            plan = self._plan_segment(current, goal)
            if plan.error_code:
                self.get_logger().warn("goal_poses: 계획 실패 (%d): %s", plan.error_code, plan.error_msg)
                return
            poses = list(plan.path.poses)
            if combined and poses:
                last = combined[-1].pose.position
                first = poses[0].pose.position
                if last.x == first.x and last.y == first.y:
                    poses = poses[1:]
            combined.extend(poses)
            current = goal
        if not combined or self._cancel_requested:
            return
        path = Path(Header(self.now(), self.global_frame.value), tuple(combined))
        self.get_logger().info("goal_poses: combined path %d poses", len(combined))
        self.publish("plan", path, QOS_LATCHED)  # viz가 전체 경로를 그리게 (마지막 구간만 그리던 문제 해결)
        follow = self._controller.send_goal(FollowPathGoal(path), timeout=5.0)
        self._follow_handle = follow
        while follow.status == GoalStatus.ACCEPTED and not self._cancel_requested:
            time.sleep(0.1)
        if self._cancel_requested:
            try:
                follow.cancel(5.0)
            except LookupError:
                pass
            self.publish("plan", Path(Header(self.now(), self.global_frame.value)), QOS_LATCHED)  # 취소 시 경로 지우기

    def _on_new_goal(self, goal: NavigateToPoseGoal) -> bool:
        self._preempt.set()
        return True

    def _plan(self, goal: PoseStamped) -> ComputePathToPoseResult:
        message = self._planner.send_goal(ComputePathToPoseGoal(goal), timeout=5.0).result(30.0)
        result = message.result
        if not isinstance(result, ComputePathToPoseResult):
            return ComputePathToPoseResult(error_code=R.UNKNOWN, error_msg=f"planner_server 결과가 없다 ({message.status.name})")
        return result

    def _execute(self, handle: ServerGoalHandle[NavigateToPoseGoal, NavigateToPoseFeedback]) -> NavigateToPoseResult:
        self._preempt.clear()
        goal, began, recoveries = handle.goal.pose, self.now(), 0
        plan = self._plan(goal)
        if plan.error_code:
            return self._fail(plan.error_code, plan.error_msg)
        follow = self._controller.send_goal(FollowPathGoal(plan.path), timeout=5.0)
        planned_at = self.now()
        while True:
            time.sleep(0.01)  # 실제 시간으로 조금씩 (따라가기는 controller_server가 한다; 여기서는 지켜보고 다시 계획만)
            if handle.is_cancel_requested or self._preempt.is_set():
                if handle.is_cancel_requested:
                    try:
                        follow.cancel(5.0)
                    except LookupError:
                        pass  # controller_server가 이미 내려갔다 (종료 중)
                return R(R.NONE, "취소" if handle.is_cancel_requested else "새 goal로 바뀌었다")
            if follow.status != GoalStatus.ACCEPTED:  # 따라가기가 끝났다
                result = follow.result(5.0).result
                code, message = (result.error_code, result.error_msg) if isinstance(result, FollowPathResult) else (R.UNKNOWN, f"controller_server 결과가 없다 ({follow.status.name})")
                if code == FollowPathResult.NONE and message != REPLACED:
                    return R(R.NONE)
                if code == FollowPathResult.NONE:
                    continue  # 내가 보낸 새 경로로 갈아탄 옛 goal: 새 follow를 이미 지켜보고 있다
                if recoveries >= self.number_of_recoveries.value:
                    return self._fail(code, message)
                recoveries += 1
                self.get_logger().warn("따라가기 실패 (%d: %s): 다시 계획한다 (%d/%d)", code, message, recoveries, self.number_of_recoveries.value)
                planned_at = -math.inf  # 바로 다시 계획
            if self.now() - planned_at >= self.replanning_period.value:
                plan = self._plan(goal)
                planned_at = self.now()
                if plan.error_code:
                    if follow.status != GoalStatus.ACCEPTED:  # 갈 길도 없고 따라가던 것도 끝났다
                        return self._fail(plan.error_code, plan.error_msg)
                else:
                    follow = self._controller.send_goal(FollowPathGoal(plan.path), timeout=5.0)  # controller가 새 경로로 갈아탄다
            handle.publish_feedback(self._feedback(goal, began, recoveries))

    def _feedback(self, goal: PoseStamped, began: float, recoveries: int) -> NavigateToPoseFeedback:
        try:
            t = self.buffer.lookup_transform(self.global_frame.value, self.robot_base_frame.value).transform
        except TransformException:
            return NavigateToPoseFeedback(navigation_time=self.now() - began, number_of_recoveries=recoveries)
        here = PoseStamped(Header(self.now(), self.global_frame.value), Pose(Point(t.translation.x, t.translation.y, 0.0), t.rotation))
        remaining = math.hypot(goal.pose.position.x - t.translation.x, goal.pose.position.y - t.translation.y)
        return NavigateToPoseFeedback(here, self.now() - began, remaining, recoveries)

    def _fail(self, code: int, message: str) -> NavigateToPoseResult:
        self.get_logger().warn("내비게이션 실패 (%d): %s", code, message)
        return R(code, message)



