from kros.topic import QoS
from kros.topic.qos import Durability

TF_TOPIC_NAME = "tf"  # 움직이는 변환 (바퀴, 오도메트리 ...)
TF_STATIC_TOPIC_NAME = "tf_static"  # 안 변하는 변환 (센서가 달린 위치 ...): 한 번만 내므로 늦게 뜬 노드도 받아야 한다 (latch)
JOINT_STATES_TOPIC_NAME = "joint_states"
ROBOT_DESCRIPTION_TOPIC_NAME = "robot_description"  # URDF 본문 (한 번만 내므로 latch)

QOS_TF = QoS(depth=100)  # 센서 값처럼 느린 구독자는 오래된 것을 건너뛰되 넉넉하게
QOS_TF_STATIC = QoS(depth=100, durability=Durability.TRANSIENT_LOCAL)  # 정적 변환 메시지를 최대 100개까지 기억해 늦게 뜬 리스너에게 준다
