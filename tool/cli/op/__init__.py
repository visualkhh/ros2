"""op 패키지 — robot tool CLI의 OOP 분리.

- terminal: 터미널 원자 연산 (크기·색·너비)
- defs: 메뉴 내비게이션 상수
- keys: 키 읽기 (cbreak · 방향키)
- component: Component — 화면 부품 최상위 부모
- window: Window — 조합·그리기·키 루프·신호 (키 규약)
- menu_bar: MenuBar → LeftSideMenuBar · ChoiceItem
- state_bar: State(칸) · StateBar · TopStateBar
- dialog: Dialog 최상위 부모 + Input·Key·Pager·Busy·Teleop
- ui: Ui — 창 진입점 · 터미널 모드 · ui.window
- state: StateStore — ROS 상태 보관 (odom·scan·map·amcl)
- procs: Procs — tool이 띄우는 launch(베이스·SLAM·Nav) 관리 + 지도 파일
- bus: Bus — 발행·액션·명령 실행
- pages: Page(기반) → Menu(Window.run) → 각 메뉴 화면
"""
