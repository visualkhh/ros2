# kos_ws — 실제 로봇용 overlay 워크스페이스

`ros2/ros2_ws`를 underlay로 쓰고, 이 워크스페이스는 새 패키지만 담는다.
시뮬레이터(kros)는 `ros2/kros`에서 별도로 실행한다.

## 사용

```bash
cd /home/pank/workspaces/visualkhh/source/ros2/kos_ws
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

## firmware

OpenCR 펌웨어: `ros2/firmware/opencr/tb3_wheels/`
Pi에서 `/dev/opencr` → `ttyACM0`
