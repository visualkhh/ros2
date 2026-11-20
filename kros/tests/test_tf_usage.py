import unittest

from kros.examples.turtlebot3.tf_usage import run
from kros.msgs.geometry_msgs import Vector3


class TestTfUsageExample(unittest.TestCase):
    """README의 TF 사용법 예제(examples/turtlebot3/tf_usage.py)를 실제로 돌려서 낡지 않는지 본다."""

    def test_example_produces_the_documented_results(self) -> None:
        result = run(2.0)
        frames = result["frames"]
        assert isinstance(frames, list)
        for frame in ("odom", "base_footprint", "base_link", "base_scan", "wheel_left_link"):
            self.assertIn(frame, frames)  # 제공 노드(RobotStatePublisher)와 내가 낸 odom이 하나의 트리로 이어졌다

        point = result["lidar_point_1m_in_base"]
        assert isinstance(point, Vector3)
        self.assertAlmostEqual(point.x, 1.0 - 0.064)  # 라이다는 로봇 중심 뒤 6.4cm
        self.assertAlmostEqual(point.z, 0.132)

        robot_in_odom, lidar_in_odom, robot_x = result["robot_in_odom"], result["lidar_in_odom"], result["robot_x"]
        assert isinstance(robot_in_odom, Vector3) and isinstance(lidar_in_odom, Vector3) and isinstance(robot_x, float)
        self.assertAlmostEqual(robot_in_odom.x, robot_x, places=6)  # 내가 낸 odom 변환
        self.assertAlmostEqual(lidar_in_odom.x, robot_x - 0.064, places=6)  # odom -> base_footprint -> base_link -> base_scan 체인
        self.assertGreater(robot_x, 0.1)  # 로봇이 실제로 움직였다

    def test_example_is_repeatable_under_simulated_time(self) -> None:
        a, b = run(1.0), run(1.0)
        self.assertEqual(a["robot_x"], b["robot_x"])  # 시뮬레이션 시간이라 같은 입력은 같은 결과


if __name__ == "__main__":
    unittest.main()
