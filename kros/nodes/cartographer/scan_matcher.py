"""cartographer의 실시간 상관 스캔 매칭(RealTimeCorrelativeScanMatcher) 흉내: 예상 위치 둘레를 격자로 뒤져 스캔 점들이 벽 칸에 가장 잘 얹히는 위치를 찾는다.
거친 간격으로 넓게 본 뒤 가장 좋은 곳 둘레를 점점 촘촘하게 본다 (모든 경우를 촘촘히 보면 느리다)."""
import math

from .probability_grid import ProbabilityGrid

Pose2 = tuple[float, float, float]  # (x, y, yaw)
STAGES = ((0.05, 0.025), (0.0125, 0.00625), (0.003, 0.0015))  # (거리 간격 m, 각 간격 rad): 단계마다 앞 단계 간격 둘레만 본다


def score(grid: ProbabilityGrid, points: list[tuple[float, float]], pose: Pose2) -> float:
    """센서가 pose에 있을 때 스캔 점(센서 좌표)들이 놓이는 칸의 평균 벽 확률."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return sum(grid.probability(x + c * px - s * py, y + s * px + c * py) for px, py in points) / len(points)


def match(grid: ProbabilityGrid, points: list[tuple[float, float]], guess: Pose2, linear_window: float = 0.1, angular_window: float = 0.1) -> tuple[Pose2, float]:
    """guess 둘레 +-linear_window(m), +-angular_window(rad) 안에서 점수가 가장 높은 센서 위치와 그 점수."""
    best, best_score = guess, score(grid, points, guess)
    window = (linear_window, angular_window)
    for linear_step, angular_step in STAGES:
        center = best
        n, m = round(window[0] / linear_step), round(window[1] / angular_step)
        for k in range(-m, m + 1):
            yaw = center[2] + k * angular_step
            c, s = math.cos(yaw), math.sin(yaw)
            rotated = [(c * px - s * py, s * px + c * py) for px, py in points]  # 각마다 한 번만 돌린다
            for i in range(-n, n + 1):
                for j in range(-n, n + 1):
                    x, y = center[0] + i * linear_step, center[1] + j * linear_step
                    value = sum(grid.probability(x + rx, y + ry) for rx, ry in rotated) / len(points)
                    if value > best_score:
                        best, best_score = (x, y, yaw), value
        window = (linear_step, angular_step)
    return best, best_score
