# turtlebot3_manipulation_description

ROBOTIS TurtleBot3 Manipulation(Waffle Pi + OpenMANIPULATOR-X) 기본 설정 + 전방 RealSense D435 마운트.
원본: `waffle-mpx-lyrical/turtlebot3_manipulation_description` (xacro·메시를 그대로 복사).

- `urdf/turtlebot3_manipulation.urdf`: 원본 xacro를 ROS 2 Lyrical `xacro`로 펼친 것 (`use_sim:=true`).
  srf의 URDF 파서는 xacro 매크로를 풀지 않으므로 이 파일을 쓴다.
- 메시 경로는 `package://turtlebot3_manipulation_description/meshes/...` → 이 폴더 기준.

다시 만들기 (원본 xacro를 고쳤을 때):

```bash
docker run --rm -v $PWD:/w ros:lyrical bash -c '
  apt-get update -qq && apt-get install -y -qq ros-lyrical-xacro >/dev/null && source /opt/ros/lyrical/setup.bash &&
  cp -r /w /tmp/p && cd /tmp/p && grep -rl "\$(find turtlebot3_manipulation_description)" urdf ros2_control gazebo |
  xargs sed -i "s#\$(find turtlebot3_manipulation_description)#/tmp/p#g" &&
  xacro urdf/turtlebot3_manipulation.urdf.xacro use_sim:=true > /w/urdf/turtlebot3_manipulation.urdf'
```
