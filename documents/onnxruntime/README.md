# ONNX Runtime GPU / CPU (RL Nav)

`krosc_ws`의 RL 주행 제어기는 ONNX Runtime으로 `policy.onnx`를 추론한다.
런타임이 GPU provider를 제공하면 CUDA를 쓰고, 없거나 초기화에 실패하면 CPU로
자동 fallback한다. 이 문서는 PC에서 **CUDA provider를 설치하고 검증하는 방법**과
실패 시 동작·소스 위치를 정리한다.

## 현재 PC 설치 상태

- GPU: `NVIDIA GeForce RTX 5060 Laptop GPU` (driver 580.178.04)
- ONNX Runtime: **1.26.0 GPU** (NuGet `Microsoft.ML.OnnxRuntime.Gpu.Linux`)
- 설치 위치: `/home/pank/opt/onnxruntime-gpu/1.26.0/`
- CUDA/cuDNN: Isaac Sim에 포함된 CUDA 12.8 / cuDNN 9 라이브러리를 symlink로 사용
- `krosc_ws` 빌드 캐시:
  - `KROSC_WS_ONNXRUNTIME_ROOT=/home/pank/opt/onnxruntime-gpu/1.26.0`
  - `ONNXRUNTIME_LIBRARY=/home/pank/opt/onnxruntime-gpu/1.26.0/lib/libonnxruntime.so`

```text
/home/pank/opt/onnxruntime-gpu/1.26.0/
├── include/       # 1.26.0 C/C++ headers
├── lib/           # libonnxruntime.so, providers_cuda.so, providers_shared.so
└── cuda/          # CUDA 12.8/cuDNN 9 dependency symlinks
```

## 설치 절차

### 1. GPU용 ONNX Runtime 내려받기

NuGet GPU Linux 패키지를 사용한다. Python wheel은 C++ header가 없으므로
C++ 프로젝트에는 NuGet 패키지가 맞다.

```bash
mkdir -p ~/opt/onnxruntime-gpu && cd /tmp
curl -fL -o ort_gpu_linux.nupkg \
  https://api.nuget.org/v3-flatcontainer/microsoft.ml.onnxruntime.gpu.linux/1.26.0/microsoft.ml.onnxruntime.gpu.linux.1.26.0.nupkg

rm -rf ort_gpu_linux && mkdir ort_gpu_linux
cd ort_gpu_linux
unzip -q ../ort_gpu_linux.nupkg \
  'buildTransitive/native/include/*' 'runtimes/linux-x64/native/*'

root=~/opt/onnxruntime-gpu/1.26.0
mkdir -p "$root/include" "$root/lib" "$root/cuda"
cp buildTransitive/native/include/*.h "$root/include/"
cp runtimes/linux-x64/native/libonnxruntime.so "$root/lib/"
cp runtimes/linux-x64/native/libonnxruntime_providers_cuda.so "$root/lib/"
cp runtimes/linux-x64/native/libonnxruntime_providers_shared.so "$root/lib/"
ln -sf libonnxruntime.so "$root/lib/libonnxruntime.so.1"
```

### 2. CUDA/cuDNN 라이브러리 연결

현재 PC는 Isaac Sim의 CUDA 12.8/cuDNN 9를 사용한다. 시스템 CUDA가 있으면
그 경로를 사용해도 된다. 아래는 현재 설치에서 사용한 대상이다.

```bash
LDP=/home/pank/install/isaac-sim-standalone-6.1.0-linux-x86_64/exts/isaacsim.pip.nv/pip_prebundle/nvidia
root=~/opt/onnxruntime-gpu/1.26.0
for d in cublas cudnn curand cufft cuda_runtime cuda_nvrtc cuda_cupti cuda_nvtx; do
  for f in "$LDP/$d/lib"/*.so*; do
    [ -e "$f" ] && ln -sf "$f" "$root/cuda/$(basename "$f")"
  done
done
# Isaac Sim의 CUDA 12 runtime이 별도 위치에 있는 경우
ln -sf /home/pank/install/isaac-sim-standalone-6.1.0-linux-x86_64/extscache/omni.gpu_foundation-0.0.0+00c488ae.lx64.r.cp312/bin/deps/libcudart.so.12 \
  "$root/cuda/libcudart.so.12"
```

### 3. CUDA provider 라이브러리에 CUDA RPATH 넣기

`libonnxruntime.so`가 provider를 `dlopen`할 때 provider의 CUDA 의존성은
실행 파일의 RPATH를 따라가지 않는다. provider 자체에 RPATH를 넣어야 한다.

```bash
# patchelf가 없으면 venv에 설치
python3 -m venv /tmp/patchelf-venv
/tmp/patchelf-venv/bin/pip install patchelf

root=~/opt/onnxruntime-gpu/1.26.0
/tmp/patchelf-venv/bin/patchelf \
  --set-rpath '$ORIGIN/../cuda:$ORIGIN' \
  "$root/lib/libonnxruntime_providers_cuda.so"
```

### 4. 빌드

`krosc_ws/build.sh`는 `~/opt/onnxruntime-gpu/1.26.0`이 있으면 자동으로
`KROSC_WS_ONNXRUNTIME_ROOT`와 `ONNXRUNTIME_ROOT`를 잡는다.
다른 경로를 쓰려면 `KROSC_WS_ONNXRUNTIME_ROOT` 환경변수로 지정한다.

```bash
# GPU 런타임 자동 감지
./krosc_ws/build.sh simulator
./krosc_ws/build.sh robot
./krosc_ws/build.sh tests
```

강제로 CPU vendor를 쓰려면:

```bash
KROSC_WS_ONNXRUNTIME_ROOT=/opt/ros/lyrical/opt/onnxruntime_vendor \
  ./krosc_ws/build.sh simulator
```

### 5. 검증

```bash
ldd krosc_ws/build/krosc_ws_simulator | grep onnxruntime
# libonnxruntime.so.1 => /home/pank/opt/onnxruntime-gpu/1.26.0/lib/libonnxruntime.so.1

readelf -d krosc_ws/build/krosc_ws_simulator | grep -E 'RPATH|RUNPATH'
# RPATH /home/pank/opt/onnxruntime-gpu/1.26.0/lib
```

시뮬레이터를 띄우고 viz-web의 **RL Nav** 탭에서 확인한다.

```bash
./krosc_ws/run-simulator-vizweb.sh
```

정상 GPU라면:

```text
provider  CUDAExecutionProvider
onnx      runtime 1.26.0
state     READY · reason 0
```

CPU fallback이면 `provider`가 `CPUExecutionProvider`로 표시된다.

## 코드에서의 처리

### Provider 선택과 fallback

[`krosc_ws/engine/ml/onnx_policy.cpp`](../../krosc_ws/engine/ml/onnx_policy.cpp)

우선순위:

```text
CUDAExecutionProvider → CoreMLExecutionProvider → CPUExecutionProvider
```

중요한 점:

- CUDA는 shared provider이므로 문자열 `AppendExecutionProvider("CUDAExecutionProvider")`로는
  잡히지 않는다. `AppendExecutionProvider_CUDA(...)`를 명시적으로 호출한다.
- CUDA provider 초기화가 `DefaultLogger`를 요구하므로 `Ort::Env`를 provider 추가보다
  먼저 만든다.
- CUDA 세션 생성이 실패하면 `provider_error_`에 원인을 남기고 CPU로 재시도한다.
- `provider_error`는 RL 상태 토픽에 `provider_error=...`로 실린다.

### RL 상태/fallback

[`krosc_ws/engine/nodes/rl_nav_controller.cpp`](../../krosc_ws/engine/nodes/rl_nav_controller.cpp)

- 모델이 없거나 로드 실패 시 RPP fallback으로 주행한다.
- `FollowPath.enable=false`면 RL은 OFF 상태이고, 웹에서 RL ON으로 켤 수 있다.
- 상태 토픽에는 `provider`, `providers`, `provider_error`, `onnxruntime`,
  `policy`, `infer_ms`, `fallback` 등이 포함된다.

### 웹 UI

[`krosc_ws/viz-web/www/plugins/rl_nav.js`](../../krosc_ws/viz-web/www/plugins/rl_nav.js)

- RL Nav 탭에서 provider/runtime/policy/fallback/model을 표시한다.
- `Reload`는 `rl_nav_controller/reload` 서비스를 호출한다.
- 모델 목록은 `artifacts/models/rl_nav_controller/`를 스캔한다.

### 빌드 설정

- [`krosc_ws/CMakeLists.txt`](../../krosc_ws/CMakeLists.txt)
  - `KROSC_WS_ONNXRUNTIME_ROOT` 기본값 자동 감지
  - `ONNXRUNTIME_INCLUDE_DIR`, `ONNXRUNTIME_LIBRARY` 탐색
  - GPU 라이브러리 사용 시 `--disable-new-dtags`로 DT_RPATH를 사용
    (시스템 `/opt/ros`의 CPU `libonnxruntime.so.1`보다 우선시키기 위함)
- [`krosc_ws/build.sh`](../../krosc_ws/build.sh)
  - `~/opt/onnxruntime-gpu/1.26.0` 또는 `ONNXRUNTIME_ROOT` 자동 사용
  - CMake 캐시의 이전 include/library 경로를 `-U`로 갱신

## 실제 예제 (_ws)

- 모델: [`krosc_ws/artifacts/models/rl_nav_controller/rl_maze/policy.onnx`](../../krosc_ws/artifacts/models/rl_nav_controller/rl_maze/policy.onnx)
- 모델 목록: `rl_maze`, `rl_maze_beebot_night1`, `rl_maze_beebot_path10`, `rl_maze_beebot_path500`
- 시뮬레이터:
  ```bash
  ./krosc_ws/run-simulator-vizweb.sh
  # http://localhost:8080/ → RL Nav 탭
  ```
- 실기:
  ```bash
  ./krosc_ws/run.sh
  ./krosc_ws/run-viz-web.sh
  ```

## 문제 해결

| 증상 | 확인 |
|---|---|
| `provider=CPUExecutionProvider` | `~/opt/onnxruntime-gpu`가 없거나 runtime이 provider를 못 찾음 |
| `provider_error=... DefaultLogger ...` | `Ort::Env` 생성 전에 CUDA provider를 붙였는지 확인 |
| `Failed to load ... libcublasLt.so.12` | provider RPATH/`cuda` symlink 확인 |
| `libonnxruntime.so.1: version VERS_1.26.0 not found` | `/opt/ros` CPU 라이브러리가 먼저 잡힘 → DT_RPATH/`--disable-new-dtags` 확인 |
| 첫 추론이 느림 | CUDA context/JIT 초기화 비용. 두 번째 추론부터 `infer_ms` 확인 |
