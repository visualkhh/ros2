# setup/pi — 로봇 Pi(`pak`) 스크립트

| 스크립트 | 하는 일 |
|---|---|
| `build.sh [colcon 인자…]` | `ros2_ws` 빌드 한 번에: 패치 되돌리기 → `vcs import`(deps.repos 고정 커밋) → `patches/*.patch` 적용 → `colcon build` (2개씩 · CMake 정책 옵션). 처음이든 `git pull` 뒤든 같은 명령 |

```bash
ssh pak
cd ~/ros2 && git pull && ./setup/pi/build.sh
```

처음 세팅의 나머지(OS · ROS apt · udev · OpenCR)는 [documents/raspberry/README.md](../../documents/raspberry/README.md) 순서대로.
패치 이름 규칙: `ros2_ws/patches/<src 폴더 이름>-<설명>.patch` → `src/<src 폴더 이름>`에 적용.
