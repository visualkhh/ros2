# 저장한 지도

viz 패널의 **save map**(`map_saver/save_map`)이 지도 하나를 폴더 하나로 저장한다. **load**는 이 폴더 목록에서 고른다.

```
maps/
└── <지도 이름>/        예: map_20261120_165654
    ├── map.yaml        ROS map_server 형식 (image, resolution, origin, thresholds)
    └── map.pgm         P5 8비트 그림 (벽 0, 빈 곳 254, 모름 205)
```

ROS의 map_server/map_saver와 같은 형식이라 서로 주고받을 수 있다. 지도 폴더들은 git에 올리지 않는다(`kros/.gitignore`), 이 README만 올린다.
