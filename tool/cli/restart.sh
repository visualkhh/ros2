#!/usr/bin/env bash
# robot CLI를 끄고 다시 켠다.
set -eo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
"$HERE/stop.sh"
exec "$HERE/start.sh" "$@"
