# kos_ws — 실제 로봇용 overlay 워크스페이스

`ros2/ros2_ws`를 **underlay**로 쓰고, 이 워크스페이스는 **새 패키지만** 담는다.
중복 빌드 없이 `ros2_ws`의 cartographer · rl_nav_controller · tb3_bringup 등을 그대로 쓴다.

## 사용

```bash
cd /home/pank/workspaces/visualkhh/source/ros2/kos_ws

# 환경 켜기 (underlay + overlay)
source ./setup.sh

# 새 패키지 만들기
cd src
ros2 pkg create --build-type ament_cmake my_robot_pkg

# 빌드
cd ..
./build.sh

# 다시 환경 켜기
source ./setup.sh
```

## 주의

- `ros2/ros2_ws`가 먼저 빌드되어 있어야 한다 (underlay).
- `src/`에 넣는 패키지는 `ros2_ws`에 없는 **새 패키지**만.
- 기존 패키지(cartographer, rl_nav_controller 등)를 수정하려면 `ros2/ros2_ws/src/`에서 작업.
