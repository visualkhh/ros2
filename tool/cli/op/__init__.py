"""op 패키지 — robot tool CLI의 OOP 분리.

- terminal: 터미널 원자 연산 (크기·색·너비)
- defs: 메뉴 내비게이션 상수
- dialogs: 키 읽기 · 확인/입력/결과창 · 메뉴 루프 (키 규약)
- ui: Ui — 4분할 프레임 + 상태바 + 다이얼로그 진입점
- state: StateStore — ROS 상태 보관 (odom·scan·map·amcl)
- procs: Procs — tool이 띄우는 launch(베이스·SLAM·Nav) 관리 + 지도 파일
- bus: Bus — 발행·액션·명령 실행
- pages: Page(기반) → Menu(선택 루프) → 각 메뉴 화면
"""
