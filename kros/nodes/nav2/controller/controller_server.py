import math
import threading
from collections.abc import Mapping

from kros.action.server_goal_handle import ServerGoalHandle
from kros.context import Context
from kros.msgs.geometry_msgs import Quaternion, Twist, Vector3
from kros.msgs.nav2_msgs import FollowPath, FollowPathFeedback, FollowPathGoal, FollowPathResult
from kros.msgs.nav_msgs import Odometry
from kros.nodes.nav2.costmap_2d import Costmap2DROS
from kros.nodes.node import Node
from kros.parameter import ParameterValue

from .controller import Controller, ControllerError, Velocity
from .dwb import DWBLocalPlanner
from .goal_checker import SimpleGoalChecker, SimpleProgressChecker
from .regulated_pure_pursuit import RegulatedPurePursuitController

PLUGINS: dict[str, type[Controller]] = {  # nav2처럼 플러그인 이름으로 알고리즘을 고른다
    "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController": RegulatedPurePursuitController,
    "dwb_core::DWBLocalPlanner": DWBLocalPlanner,
}
R = FollowPathResult
CMD_VEL_TOPIC_NAME = "cmd_vel"


class ControllerServer(Node):
    """nav2 controller_server 흉내: follow_path 액션(nav2_msgs/FollowPath)으로 경로를 받아 controller_frequency마다 cmd_vel을 낸다.
    nav2처럼 자기 안에 local_costmap(Costmap2DROS) 노드를 만든다 (파라미터는 costmap_parameters).
    shortcut: local_costmap은 nav2의 rolling window + obstacle layer가 아니라 지도 기반이다, 지도에 없는 장애물을 피해야 할 때 바꾼다.

    파라미터: controller_frequency (Hz), controller_plugins (기본 ("FollowPath",)), "<이름>.plugin" (PLUGINS의 키), odom_topic (지금 속도),
    general_goal_checker.xy_goal_tolerance/yaw_goal_tolerance (도착), progress_checker.required_movement_radius/movement_time_allowance (막힘).
    새 경로(goal)가 오면 앞의 것은 물러나고 새 경로를 따른다 (nav2가 재계획한 경로로 갈아타듯).
    """

    def __init__(
        self, context: Context, name: str | None = None, parameters: Mapping[str, ParameterValue] | None = None, costmap_parameters: Mapping[str, ParameterValue] | None = None
    ) -> None:
        super().__init__(context, name or "controller_server", parameters)
        self.costmap_ros = Costmap2DROS(context, "local_costmap", costmap_parameters)
        self.controller_frequency = self.declare_parameter("controller_frequency", 20.0, validator=lambda value: value > 0)
        self.controller_plugins = self.declare_parameter("controller_plugins", ("FollowPath",), validator=lambda ids: len(ids) > 0)
        self.odom_topic = self.declare_parameter("odom_topic", "odom")
        self.controllers: dict[str, Controller] = {}
        for plugin_id in self.controller_plugins.value:
            kind = self.declare_parameter(f"{plugin_id}.plugin", "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController", validator=lambda value: value in PLUGINS)
            controller = PLUGINS[kind.value]()
            controller.configure(self, str(plugin_id))
            self.controllers[str(plugin_id)] = controller
        self.goal_checker = SimpleGoalChecker(self)
        self.progress_checker = SimpleProgressChecker(self)
        self.velocity: Velocity = (0.0, 0.0)
        self._tick = threading.Event()  # 제어 주기마다 (시뮬레이션 시간이면 시뮬레이션 시간으로) 켜진다
        self._preempt = threading.Event()  # 새 goal이 들어왔다: 지금 goal은 물러난다 (액션 서버는 goal을 하나씩 차례로 돌린다)
        self.subscribe(self.odom_topic.value, Odometry, self._on_odom)
        self.create_timer(1.0 / self.controller_frequency.value, self._tick.set)
        self.create_action_server("follow_path", FollowPath, self._execute, goal_callback=self._on_new_goal)

    def _on_new_goal(self, goal: FollowPathGoal) -> bool:
        self._preempt.set()  # 대기열에 들어가기 전에 불린다: 돌고 있는 goal이 이걸 보고 끝나야 새 goal이 시작된다
        return True

    def _on_odom(self, odom: Odometry) -> None:
        self.velocity = (odom.twist.twist.linear.x, odom.twist.twist.angular.z)

    def _stop(self) -> None:
        self.publish(CMD_VEL_TOPIC_NAME, Twist())

    def _execute(self, handle: ServerGoalHandle[FollowPathGoal, FollowPathFeedback]) -> FollowPathResult:
        goal = handle.goal
        controller_id = goal.controller_id or str(self.controller_plugins.value[0])
        controller = self.controllers.get(controller_id)
        if controller is None:
            return self._fail(R.INVALID_CONTROLLER, f"컨트롤러 {controller_id!r}가 없다 (있는 것: {', '.join(self.controllers)})")
        frame = self.costmap_ros.global_frame.value
        if not goal.path.poses or goal.path.header.frame_id not in ("", frame):
            return self._fail(R.INVALID_PATH, f"빈 경로이거나 {frame} 좌표계가 아니다 ({goal.path.header.frame_id})")
        path = [(p.pose.position.x, p.pose.position.y, _yaw(p.pose.orientation)) for p in goal.path.poses]
        self._preempt.clear()
        self.progress_checker.reset()
        period = 1.0 / self.controller_frequency.value
        while True:
            self._tick.clear()
            while not self._tick.wait(0.2):  # 다음 제어 주기까지 (취소/교체는 주기마다 본다)
                if handle.is_cancel_requested or self._preempt.is_set():
                    break
            if self._preempt.is_set():
                return R(R.NONE, "새 경로로 바뀌었다")  # cmd_vel은 새 goal이 이어서 낸다
            if handle.is_cancel_requested:
                self._stop()
                return R(R.NONE, "취소")
            costmap, pose = self.costmap_ros.costmap, self.costmap_ros.get_robot_pose()
            if costmap is None or pose is None:
                return self._fail(R.TF_ERROR, "local_costmap에 지도가 없거나 로봇 위치(TF)를 모른다")
            if self.goal_checker.is_goal_reached(pose, path[-1]):
                self._stop()
                return R(R.NONE)
            if not self.progress_checker.check(pose, self.now()):
                return self._fail(R.FAILED_TO_MAKE_PROGRESS, f"{self.progress_checker.movement_time_allowance.value}초 동안 움직이지 못했다")
            try:
                v, w = controller.compute_velocity_commands(pose, self.velocity, path, costmap, period)
            except ControllerError as error:
                return self._fail(error.code, str(error))
            self.publish(CMD_VEL_TOPIC_NAME, Twist(Vector3(x=v), Vector3(z=w)))
            handle.publish_feedback(FollowPathFeedback(math.hypot(path[-1][0] - pose[0], path[-1][1] - pose[1]), v))

    def _fail(self, code: int, message: str) -> FollowPathResult:
        self._stop()
        self.get_logger().warn("경로 따라가기 실패 (%d): %s", code, message)
        return R(code, message)


def _yaw(q: Quaternion) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
