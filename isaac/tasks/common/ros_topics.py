"""Isaac Sim ↔ ROS 2 토픽 계약 — bridge·평가 노드가 같이 쓰는 한 곳.

USD(로봇 자산)에는 토픽이 없다. 토픽은 이 파일이 정본이고, bridge가 여기 이름으로
publish/subscribe한다. real(pak)은 ROS_DOMAIN_ID=25, Isaac sim은 26으로 분리한다.

주의: `/isaacros/ground_truth/*`는 평가 전용. 정책 obs에 넣지 않는다.
"""

NS = "isaacros"
DOMAIN_ID = 26          # real = 25. sim과 real을 동시에 켜도 DDS가 안 섞이게.

CLOCK = f"/{NS}/clock"
TF = f"/{NS}/tf"
TF_STATIC = f"/{NS}/tf_static"
JOINT_STATES = f"/{NS}/joint_states"
ODOM = f"/{NS}/odom"
IMU = f"/{NS}/imu"
SCAN = f"/{NS}/scan"
CMD_VEL = f"/{NS}/cmd_vel"
BATTERY = f"/{NS}/battery_state"
MAP = f"/{NS}/map"
GROUND_TRUTH_POSE = f"/{NS}/ground_truth/pose"
RESET = f"/{NS}/reset"

# (토픽, 타입, 방향, 프레임/비고)
TOPICS = {
    CLOCK: ("rosgraph_msgs/msg/Clock", "pub", "use_sim_time=true인 모든 노드가 구독"),
    TF: ("tf2_msgs/msg/TFMessage", "pub", "map→odom→base_footprint→base_link"),
    TF_STATIC: ("tf2_msgs/msg/TFMessage", "pub", "base_link→imu_link, base_scan"),
    JOINT_STATES: ("sensor_msgs/msg/JointState", "pub", "wheel_left/right_joint"),
    ODOM: ("nav_msgs/msg/Odometry", "pub", "frame odom · child base_footprint"),
    IMU: ("sensor_msgs/msg/Imu", "pub", "frame imu_link"),
    SCAN: ("sensor_msgs/msg/LaserScan", "pub", "frame base_scan · 360°"),
    CMD_VEL: ("geometry_msgs/msg/Twist", "sub", "Nav2/텔레오퍼 명령"),
    BATTERY: ("sensor_msgs/msg/BatteryState", "pub", "12V"),
    MAP: ("nav_msgs/msg/OccupancyGrid", "pub", "SLAM 단계에서"),
    GROUND_TRUTH_POSE: ("geometry_msgs/msg/PoseStamped", "pub", "평가 전용 — 정책 입력 금지"),
    RESET: ("std_srvs/srv/Empty", "sub", "에피소드 리셋 (L4 이후)"),
}

# real 로봇과 같은 TF 이름을 쓴다 (frame prefix를 붙이지 않음 — domain으로 분리)
FRAMES = ("map", "odom", "base_footprint", "base_link", "imu_link", "base_scan")
