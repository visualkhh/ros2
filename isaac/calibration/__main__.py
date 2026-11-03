"""python -m calibration <sim | real | dummy> [옵션...] — calibration/run.py (런타임별 import는 그 안에서 lazy)."""
from calibration.run import main

raise SystemExit(main() or 0)
