from machine import UART, Pin
import time

# Arduino Nano ESP32 ↔ XBee
# D1(GPIO43) → XBee DIN
# D0(GPIO44) ← XBee DOUT
xbee = UART(
    1,
    baudrate=9600,
    tx=Pin(43),
    rx=Pin(44),
    bits=8,
    parity=None,
    stop=1,
    timeout=1000,
)

def read_response(wait_ms=400):
    time.sleep_ms(wait_ms)
    data = b""

    while xbee.any():
        chunk = xbee.read()
        if chunk:
            data += chunk
        time.sleep_ms(50)

    return data

# 기존 수신 데이터 제거
while xbee.any():
    xbee.read()

print("XBee 설정 확인 시작")
print("명령 모드 진입 대기 중...")

# +++ 앞뒤에는 다른 데이터를 보내면 안 됩니다.
time.sleep_ms(1200)
xbee.write(b"+++")
response = read_response(1200)

print("명령 모드 응답:", response)

if response and b"OK" in response:
    commands = [
        ("ATHV", "하드웨어 버전"),
        ("ATVR", "펌웨어 버전"),
        ("ATCE", "Coordinator 여부"),
        ("ATID", "PAN ID"),
        ("ATAP", "API 모드"),
        ("ATBD", "UART 속도"),
        ("ATSH", "주소 상위"),
        ("ATSL", "주소 하위"),
        ("ATCH", "현재 채널"),
        ("ATAI", "네트워크 연결 상태"),
    ]

    for command, description in commands:
        xbee.write((command + "\r").encode())
        value = read_response()
        print(command, description, "=", value)

    xbee.write(b"ATCN\r")
    print("AT 명령 모드 종료:", read_response())
    print("확인 완료")
else:
    print("XBee 응답 없음")
    print("배선, 전원, 9600 baud 및 XBee 투명 모드를 확인하세요.")
