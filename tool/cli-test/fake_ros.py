"""ROS 없는 환경에서 visualkhh robot_tool UI/UX를 시험하는 가짜 ROS층.

진짜 robot_tool 코드를 그대로 import한다. 발행은 [PUB ...]으로 찍고,
/state_*·/cmd_manual_*는 가짜 state_manager처럼 상태를 바꾼다.
"""
from __future__ import annotations

import sys
import types


class _Msg:
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


class Header(_Msg):
    def __init__(self, **kw):
        self.stamp = _Time()
        super().__init__(**kw)


class _Time:
    sec = 0
    nanosec = 0

    def to_msg(self):
        return self


class String(_Msg):
    def __init__(self, **kw):
        self.data = ''
        super().__init__(**kw)

    def __repr__(self):
        return f"String(data={self.data!r})"


class Bool(_Msg):
    def __init__(self, **kw):
        self.data = False
        super().__init__(**kw)


class Empty(_Msg):
    pass


class Int32MultiArray(_Msg):
    def __init__(self, data=None, **kw):
        self.data = list(data or [])
        super().__init__(**kw)


class Float32MultiArray(_Msg):
    def __init__(self, data=None, **kw):
        self.data = list(data or [])
        super().__init__(**kw)


class Vector3(_Msg):
    def __init__(self, **kw):
        self.x = self.y = self.z = 0.0
        super().__init__(**kw)


class Twist(_Msg):
    def __init__(self, **kw):
        self.linear = Vector3()
        self.angular = Vector3()
        super().__init__(**kw)


class TwistStamped(_Msg):
    def __init__(self, **kw):
        self.header = Header()
        self.twist = Twist()
        super().__init__(**kw)

    def __repr__(self):
        t = self.twist
        return (f'TwistStamped(linear={t.linear.x:.2f}, angular={t.angular.z:.2f})')


class Point(_Msg):
    def __init__(self, **kw):
        self.x = self.y = self.z = 0.0
        super().__init__(**kw)


class Quaternion(_Msg):
    def __init__(self, **kw):
        self.x = self.y = self.z = 0.0
        self.w = 1.0
        super().__init__(**kw)


class Pose(_Msg):
    def __init__(self, **kw):
        self.position = Point()
        self.orientation = Quaternion()
        super().__init__(**kw)


class PoseWithCovariance(_Msg):
    def __init__(self, **kw):
        self.pose = Pose()
        self.covariance = [0.0] * 36
        super().__init__(**kw)


class PoseWithCovarianceStamped(_Msg):
    def __init__(self, **kw):
        self.header = Header()
        self.pose = PoseWithCovariance()
        super().__init__(**kw)


class PoseStamped(_Msg):
    def __init__(self, **kw):
        self.header = Header()
        self.pose = Pose()
        super().__init__(**kw)


class Odometry(_Msg):
    def __init__(self, **kw):
        self.header = Header()
        self.pose = PoseWithCovariance()
        self.twist = Twist()  # 실제 메시지와 모양이 달라도 fake 상태 주입에는 충분
        super().__init__(**kw)


class OccupancyGrid(_Msg):
    def __init__(self, **kw):
        self.header = Header()
        self.info = _Msg(width=100, height=100, resolution=0.05)
        self.data = []
        super().__init__(**kw)


class LaserScan(_Msg):
    def __init__(self, **kw):
        self.header = Header()
        self.angle_min = -3.14
        self.angle_max = 3.14
        self.angle_increment = 0.01
        self.range_min = 0.12
        self.range_max = 3.5
        self.ranges = []
        super().__init__(**kw)


class JointState(_Msg):
    def __init__(self, **kw):
        self.header = Header()
        self.name = []
        self.position = []
        self.velocity = []
        self.effort = []
        super().__init__(**kw)


class KeyValue(_Msg):
    def __init__(self, key='', value='', **kw):
        self.key = key
        self.value = value
        super().__init__(**kw)


class DiagnosticStatus(_Msg):
    def __init__(self, **kw):
        self.level = 0
        self.name = ''
        self.message = ''
        self.values = []
        super().__init__(**kw)


class DiagnosticArray(_Msg):
    def __init__(self, **kw):
        self.header = Header()
        self.status = []
        super().__init__(**kw)


class Imu(_Msg):
    pass


class ParameterType:
    PARAMETER_NOT_SET = 0
    PARAMETER_BOOL = 1
    PARAMETER_INTEGER = 2
    PARAMETER_DOUBLE = 3
    PARAMETER_STRING = 4
    PARAMETER_BYTE_ARRAY = 5
    PARAMETER_BOOL_ARRAY = 6
    PARAMETER_INTEGER_ARRAY = 7
    PARAMETER_DOUBLE_ARRAY = 8
    PARAMETER_STRING_ARRAY = 9


class ParameterValue(_Msg):
    def __init__(self, **kw):
        self.bool_value = False
        self.integer_value = 0
        self.double_value = 0.0
        self.string_value = ''
        self.byte_array_value = []
        self.bool_array_value = []
        self.integer_array_value = []
        self.double_array_value = []
        self.string_array_value = []
        super().__init__(**kw)


class _Logger:
    def __init__(self, name):
        self.name = name

    def info(self, msg, *a, **k):
        print(f'[INFO {self.name}] {msg}')

    def warn(self, msg, *a, **k):
        print(f'[WARN {self.name}] {msg}')

    def error(self, msg, *a, **k):
        print(f'[ERR {self.name}] {msg}')

    def debug(self, msg, *a, **k):
        pass


class _Clock:
    def now(self):
        return _Time()


class _Publisher:
    def __init__(self, node, topic):
        self._node = node
        self.topic = topic

    def publish(self, msg):
        print(f'[PUB {self.topic}] {msg}')
        cb = getattr(self._node, '_fake_on_publish', None)
        if cb:
            cb(self.topic, msg)

    def get_subscription_count(self):
        return 1

    def destroy(self):
        pass


class _Subscription:
    def destroy(self):
        pass


class _Future:
    def __init__(self, result=None):
        self._result = result
        self._done = True

    def done(self):
        return self._done

    def result(self):
        return self._result


class _Client:
    def __init__(self, srv_type, topic):
        self.srv_type = srv_type
        self.topic = topic

    def wait_for_service(self, timeout_sec=1.0):
        return True

    def call_async(self, req):
        print(f'[SVC {self.topic}] 요청 → 가짜 성공')
        return _Future(_Msg(success=True, message='fake'))

    def destroy(self):
        pass


class QoSProfile:
    def __init__(self, **kw):
        self.depth = kw.get('depth', 10)


class _QosEnum:
    RELIABLE = 1
    BEST_EFFORT = 2
    TRANSIENT_LOCAL = 3
    VOLATILE = 4
    KEEP_LAST = 5


class Node:
    """RobotTool(Node)의 부모 — 필요한 API만 가짜로."""

    def __init__(self, name):
        self._fake_name = name
        self._logger = _Logger(name)

    def get_name(self):
        return self._fake_name

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return _Clock()

    def create_publisher(self, msg_type, topic, qos):
        return _Publisher(self, topic)

    def create_subscription(self, msg_type, topic, callback, qos):
        return _Subscription()

    def create_client(self, srv_type, topic):
        return _Client(srv_type, topic)

    def destroy_client(self, client):
        pass

    def destroy_node(self):
        pass

    def get_node_names(self):
        return ['robot_tool', 'controller_server',
                'opencr_driver', 'scan_resample', 'amcl', 'bt_navigator',
                'controller_manager']

    def get_node_names_and_namespaces(self):
        return [(n, '/') for n in self.get_node_names()]

    def get_publisher_names_and_types_by_node(self, name, ns):
        if name == 'controller_server':
            return [('/controller_server/FollowPath/status', ('std_msgs/msg/Float32MultiArray',))]
        return []

    def get_subscriber_names_and_types_by_node(self, name, ns):
        return []

    def count_publishers(self, topic):
        return 1 if topic == '/controller_server/FollowPath/status' else 0

    def get_publishers_info_by_topic(self, topic):
        info = _Msg(node_name='controller_server', node_namespace='/')
        return [info] if topic == '/controller_server/FollowPath/status' else []

    def _fake_on_publish(self, topic, msg):
        state = getattr(self, 'state', None)
        if state is None:
            return
        data = getattr(msg, 'data', None)
        if topic == '/cmd_manual_control' and isinstance(data, str):
            state.st_control = data
        elif topic == '/cmd_manual_estop' and isinstance(data, bool):
            state.st_estop = data


class AsyncParameterClient:
    def __init__(self, node, fqn):
        self._node = node
        self._fqn = fqn

    def wait_for_services(self, timeout_sec=1.0):
        return False


class Parameter:
    def __init__(self, name, value):
        self.name = name
        self.value = value


class ActionClient:
    def __init__(self, node, action_type, name):
        self._node = node
        self._name = name

    def wait_for_server(self, timeout_sec=5.0):
        return True

    def send_goal_async(self, goal):
        print(f'[ACTION {self._name}] goal → fake')
        return _Future(_Msg(accepted=True))

    def destroy(self):
        pass


class Trigger:
    class Request:
        pass

    class Response:
        def __init__(self, success=True, message='fake'):
            self.success = success
            self.message = message


class SetBool:
    class Request:
        def __init__(self, data=False):
            self.data = data

    class Response:
        def __init__(self, success=True, message='fake'):
            self.success = success
            self.message = message


class NavigateToPose:
    class Goal:
        def __init__(self):
            self.pose = PoseStamped()


def _mod(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    return m


def install():
    _mod('rclpy', init=lambda args=None: None, ok=lambda: True, shutdown=lambda: None,
         spin_once=lambda node, timeout_sec=0.0: None,
         spin_until_future_complete=lambda node, fut, timeout_sec=1.0: None)
    _mod('rclpy.node', Node=Node)
    _mod('rclpy.parameter', Parameter=Parameter)
    _mod('rclpy.parameter_client', AsyncParameterClient=AsyncParameterClient)
    _mod('rclpy.action', ActionClient=ActionClient)
    qos = _mod('rclpy.qos')
    qos.QoSProfile = QoSProfile
    qos.QoSReliabilityPolicy = _QosEnum
    qos.QoSDurabilityPolicy = _QosEnum
    qos.QoSHistoryPolicy = _QosEnum
    qos.qos_profile_sensor_data = QoSProfile(depth=5)

    _mod('std_msgs', msg=_mod('std_msgs.msg', String=String, Bool=Bool, Empty=Empty,
                              Int32MultiArray=Int32MultiArray, Float32MultiArray=Float32MultiArray))
    _mod('geometry_msgs', msg=_mod('geometry_msgs.msg', TwistStamped=TwistStamped, Twist=Twist,
                                  Vector3=Vector3, PoseStamped=PoseStamped,
                                  PoseWithCovarianceStamped=PoseWithCovarianceStamped,
                                  Pose=Pose, PoseWithCovariance=PoseWithCovariance,
                                  Point=Point, Quaternion=Quaternion))
    _mod('nav_msgs', msg=_mod('nav_msgs.msg', Odometry=Odometry, OccupancyGrid=OccupancyGrid))
    _mod('sensor_msgs', msg=_mod('sensor_msgs.msg', LaserScan=LaserScan, JointState=JointState,
                                 Imu=Imu, Header=Header))
    _mod('diagnostic_msgs', msg=_mod('diagnostic_msgs.msg', DiagnosticArray=DiagnosticArray,
                                    DiagnosticStatus=DiagnosticStatus, KeyValue=KeyValue))
    _mod('rcl_interfaces', msg=_mod('rcl_interfaces.msg', ParameterType=ParameterType,
                                    ParameterValue=ParameterValue))
    _mod('std_srvs', srv=_mod('std_srvs.srv', Trigger=Trigger, SetBool=SetBool))
    _mod('nav2_msgs', action=_mod('nav2_msgs.action', NavigateToPose=NavigateToPose))
