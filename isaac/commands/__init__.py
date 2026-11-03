"""명령 경로 (Isaac Lab의 command 개념): 시각 t → 명령 (v, w). env는 명령을 받아 바퀴를 돌릴 뿐이다.

BaseCommand를 상속해 만든다. base · square는 표준 라이브러리만 (실기기 측정이 Isaac 없이 ROS에서 돈다).
keyboard는 Isaac 창이 필요 (AppLauncher 뒤에 import).
"""
