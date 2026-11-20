from kros.hardwares.hardware import Hardware


class Controller(Hardware):
    """제어 보드 인터페이스: 센서와 구동기(모터 ...)를 부품으로 묶어 하나의 장치로 다루는 보드 (OpenCR, 아두이노 ...).
    부품은 보드에 달리므로 보드를 끄면 부품도 같이 꺼진다 (Hardware의 전원 규칙). 어떤 부품을 어떻게 묶는지는 모델(구현)이 정한다."""
