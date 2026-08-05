#!/usr/bin/env python3
import time
import serial

PORT = "/dev/serial0"
BAUDRATE = 9600


def read_response(ser, wait=0.4):
    """잠시 기다린 뒤 들어온 응답을 모두 읽습니다."""
    time.sleep(wait)
    data = ser.read(ser.in_waiting or 1)

    # 뒤늦게 도착한 데이터까지 추가로 읽기
    time.sleep(0.1)
    if ser.in_waiting:
        data += ser.read(ser.in_waiting)

    return data


def send_at(ser, command):
    ser.write((command + "\r").encode())
    ser.flush()
    return read_response(ser)


def main():
    print(f"XBee 설정 확인 시작: {PORT}, {BAUDRATE} baud")

    with serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1,
    ) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        print("AT 명령 모드 진입 중...")

        # +++ 전후에는 다른 데이터를 보내면 안 됩니다.
        time.sleep(1.2)
        ser.write(b"+++")
        ser.flush()
        response = read_response(ser, 1.2)

        print("명령 모드 응답:", response)

        if b"OK" not in response:
            print("XBee 응답 없음")
            print("배선, 전원, 9600 baud, 투명 모드(AP=0)를 확인하세요.")
            return

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
            value = send_at(ser, command)
            print(f"{command} {description} = {value!r}")

        print("AT 명령 모드 종료:", send_at(ser, "ATCN"))
        print("확인 완료")


if __name__ == "__main__":
    try:
        main()
    except serial.SerialException as error:
        print("직렬 포트 오류:", error)
        print("/dev/serial0을 사용하는 기존 프로그램을 먼저 종료하세요.")
    except PermissionError:
        print("권한 오류: 현재 계정에 /dev/serial0 접근 권한이 없습니다.")
