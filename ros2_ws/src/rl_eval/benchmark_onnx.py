#!/usr/bin/env python3
"""ONNX policy latency benchmark (no ROS needed).

Usage: python3 benchmark_onnx.py model.onnx [--iters 1000]
Run this ON the Raspberry Pi to decide if onboard inference is viable.
Target: mean < 50ms for a 10-20Hz control loop (small MLP is usually < 5ms on Pi 4B).
"""
import sys
import time

import numpy as np


def main():
    path = sys.argv[1]
    iters = int(sys.argv[3]) if len(sys.argv) > 3 and sys.argv[2] == '--iters' else 1000
    import onnxruntime as ort
    sess = ort.InferenceSession(path, providers=['CPUExecutionProvider'])
    name = sess.get_inputs()[0].name
    x = np.zeros((1, 2), dtype=np.float32)
    for _ in range(50):  # warmup
        sess.run(None, {name: x})
    t0 = time.perf_counter()
    for _ in range(iters):
        sess.run(None, {name: x})
    dt = (time.perf_counter() - t0) / iters * 1000
    print(f'mean inference: {dt:.2f} ms over {iters} iters ({path})')


if __name__ == '__main__':
    main()
