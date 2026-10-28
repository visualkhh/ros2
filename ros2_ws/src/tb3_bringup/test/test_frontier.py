import numpy as np

from tb3_bringup.frontier_explorer_node import frontier_clusters


def test_wall_splits_frontier():
    g = np.full((20, 20), -1, dtype=np.int8)   # 모르는 칸
    g[5:15, 5:15] = 0                           # 빈 10x10
    g[5:15, 10] = 100                           # 가운데 벽 → 좌우 두 덩어리
    sizes = sorted(len(c) for c in frontier_clusters(g, 3))
    assert sizes == [16, 18]   # 오른쪽 4칸 폭: 세로 10 + 위·아래 3씩 · 왼쪽 5칸 폭: 10 + 4 + 4


def test_fully_known_has_no_frontier():
    g = np.zeros((10, 10), dtype=np.int8)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    assert frontier_clusters(g, 1) == []
