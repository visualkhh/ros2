#!/usr/bin/env bash
# kros 테스트 실행 + 타입 검사(mypy --strict). 사용: ./test.sh [unittest 인자...]   예: ./test.sh -v -k pub_sub
# .venv가 없으면 먼저 만든다 (이 기기에 맞는 프로필로). 인자를 주면(특정 테스트만) 타입 검사는 건너뛴다.
# 타입 검사는 mypy가 설치돼 있을 때만 돈다 (sim/gpu 프로필에는 설치되고, 라즈베리파이용 core 프로필에는 없다).
here="$(cd "$(dirname "$0")" && pwd)"
[ -x "$here/.venv/bin/python" ] || "$here/setting-venv.sh" >/dev/null || { echo "venv 준비 실패" >&2; exit 1; }
cd "$here/.."
status=0
"$here/.venv/bin/python" -m unittest discover -s kros/tests -t . "$@" || status=1
if [ $# -eq 0 ]; then
  if "$here/.venv/bin/python" -c "import mypy" 2>/dev/null; then
    "$here/.venv/bin/python" -m mypy --config-file kros/mypy.ini kros || status=1
  else
    echo "(mypy가 없어 타입 검사를 건너뜀: pip install -r kros/requirements-dev.txt)"
  fi
fi
exit $status
