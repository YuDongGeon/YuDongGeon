# -*- coding: utf-8 -*-
"""
main_v2.py – 라즈베리파이5 BLE Peripheral + RF433 수신 (GATT Server)
=====================================================================

[v2 변경사항 – RPi5_REVIEW.md 반영]
  - 미사용 import 제거 (time, async_tools)
  - Status TX Characteristic (ffe2) Notify 추가 (보고서 규격 반영)
  - BLE 어댑터 감지 시 에러 핸들링 추가
  - Graceful Shutdown (signal 핸들러) 추가
  - TODO 주석에 RPi5 호환 라이브러리 안내 (gpiozero/lgpio)

[v2.1 변경사항 – RF433 리모컨 통합]
  - 확장 Command ID 추가 (방향 제어 0x11/0x21/0x31,
    프리셋 0xA0/0xA1, 최소 높이 0xB0/0xB1)
  - RF433 수신 스레드 추가 (rpi-rf 기반, 24bit → [CMD_ID, VALUE] 파싱)
  - BLE와 RF가 동일한 handle_command() 함수를 호출하여 코드 통합
  - 프리셋 저장/실행 로직 (JSON 파일 영속 저장)
  - 최소 높이 설정/초기화 로직 (모터별 독립)

[v2.2 변경사항 – 엔코더 미감지 및 무한 구동 방지]
  - lgpio.callback() 반환 객체를 self.cb에 저장
    이유: 콜백 객체의 참조가 사라져 엔코더 인터럽트가 정상 유지되지 않는
          가능성을 줄이기 위함.
  - 엔코더 ISR에서 count의 0~total_pulses 제한을 제거
    이유: 엔코더 방향이 반대로 잡힌 경우 count가 음수로 표시되도록 하여
          배선 또는 방향 설정 문제를 로그에서 확인할 수 있게 하기 위함.
          위치값(pos)은 기존과 동일하게 0~100% 범위로 제한함.
  - 모터별 엔코더 방향 반전 설정 추가
    이유: 모터 회전 방향, A/B상 배선에 따라 카운트 증가 방향이 달라질 수 있으므로
          코드 상단 설정만으로 방향을 보정할 수 있게 하기 위함.
  - 위치 제어 시작 시간 및 마지막 엔코더 변화 시간을 기록
    이유: 엔코더 펄스가 들어오지 않거나 목표 위치에 도달하지 못할 때
          모터가 무한히 동작하는 것을 방지하기 위함.
  - 엔코더 무변화 시간 초과 및 최대 구동시간 초과 시 강제 정지 추가
    이유: 엔코더 배선 불량, 센서 고장, 잘못된 방향 설정 등의 상황에서도
          모터와 기구물이 계속 구동되지 않도록 하기 위함.
  - 기존 BLE, RF433, XBee, 프리셋, 명령 처리 방식과 기존 주석은 유지함.

역할:
  라즈베리파이5를 BLE Peripheral(서버)로 동작시키고,
  동시에 RF433 수신기로 리모컨 명령을 수신한다.
  스마트폰 앱과 RF 리모컨 모두 동일한 명령 체계([CMD_ID, VALUE])를 사용하여
  하나의 명령 핸들러(handle_command)에서 통합 처리한다.

입력 경로:
  1. BLE (스마트폰 앱)  → on_write() → handle_command()
  2. RF433 (리모컨)     → rf_receive_loop() → handle_command()

통신 프로토콜:
  - 패킷 길이  : 2바이트 고정 [CMD_ID, VALUE]

  기존 (앱 + 리모컨 공용):
    0x00      = 긴급 정지
    0x01~0x03 = 목표 위치 제어 (0~100%)

  확장 (리모컨 전용):
    0x11/0x21/0x31 = 방향 제어 (UP=0x01, DOWN=0x02, STOP=0x00)
    0xA0/0xA1      = 프리셋 저장/실행 (Value: 1~3)
    0xB0/0xB1      = 최소 높이 설정/초기화 (Value: motor_id)

의존성:
  - bluezero  : D-Bus 기반 BLE GATT 서버 라이브러리
                (pip install bluezero)
  - rpi-rf    : RF433 수신 라이브러리 (pip install rpi-rf)
                ⚠️ RPi5에서는 lgpio 백엔드 필요 (내부 RPi.GPIO 호출 주의)
  - BlueZ     : Linux Bluetooth 프로토콜 스택
                ⚠️ --experimental 플래그 필수

관련 파일:
  - bleManager_v2.ts    : 스마트폰 앱 BLE 클라이언트
  - remote_control_v2.ino : RF433 리모컨 (아두이노)
"""

import sys
import signal
import threading
import json
import os
import time
import serial                      # pip install pyserial (XBee UART)
import lgpio                       # pip install lgpio (Encoder ISR)
from gpiozero import PWMOutputDevice, DigitalOutputDevice # Motor Control
from bluezero import peripheral    # BLE Peripheral(GATT 서버) 구현 모듈
from bluezero import adapter       # 시스템 BLE 어댑터(hci0) 접근 모듈

# ==========================================
# GATT Service / Characteristic UUID 정의
# ==========================================
# 스마트폰 앱(bleManager.ts)과 반드시 동일한 UUID를 사용해야 한다.
# 앱에서 이 UUID로 서비스를 검색하고 Characteristic에 데이터를 기록한다.
#
# SERVICE_UUID     : 이 BLE 장치가 제공하는 서비스의 고유 식별자.
#                    앱은 이 UUID를 기준으로 장치를 필터링/검색한다.
# CMD_CHAR_UUID    : 명령 수신용 Characteristic (Command RX).
#                    앱은 이 Characteristic에 2바이트 패킷을 Write하여 명령을 보낸다.
# STATUS_CHAR_UUID : 상태 피드백용 Characteristic (Status TX).
#                    RPi가 모터 상태/센서 데이터를 앱에 Notify로 전송한다.
#                    (1차 캡스톤 최종보고서 2-3-2절 규격)
SERVICE_UUID     = '0000ffe0-0000-1000-8000-00805f9b34fb'
CMD_CHAR_UUID    = '0000ffe1-0000-1000-8000-00805f9b34fb'
STATUS_CHAR_UUID = '0000ffe2-0000-1000-8000-00805f9b34fb'

# ==========================================
# Command ID 상수 정의 (BLE 2-Byte 프로토콜)
# ==========================================
# 각 Command ID는 제어 대상(모터/블라인드)을 식별한다.
# 패킷의 Byte 0에 해당하며, Byte 1(Value)과 조합하여 명령을 구성한다.
#
# 예시 패킷:
#   [0x01, 0x32] → 상단 롤러 블라인드를 50% 위치로 이동
#   [0x00, 0x00] → 모든 모터 긴급 정지 (Value는 무시됨)
# ── 기존 BLE 호환 (앱 + 리모컨 공용) ──
CMD_STOP    = 0x00  # 긴급 정지: 모든 모터를 즉시 멈춤 (Value 무시)
CMD_UPPER   = 0x01  # 상단 롤러 블라인드: Value(0~100%)만큼 개폐
CMD_LOWER_F = 0x02  # 하단 프레임: Value(0~100%)만큼 위치 이동
CMD_LOWER_B = 0x03  # 하단 롤러 블라인드: Value(0~100%)만큼 개폐

# ── 방향 제어 (리모컨 전용 확장) ──
CMD_DIR_UPPER   = 0x11  # 상단 방향 제어 (Value: UP=0x01, DOWN=0x02, STOP=0x00)
CMD_DIR_LOWER_F = 0x21  # 하단 프레임 방향 제어
CMD_DIR_LOWER_B = 0x31  # 하단 블라인드 방향 제어

# ── 방향 Value 상수 ──
DIR_STOP = 0x00
DIR_UP   = 0x01
DIR_DOWN = 0x02

# ── 프리셋 명령 ──
CMD_PRESET_SAVE = 0xA0  # 프리셋 저장 (Value: 1~3)
CMD_PRESET_EXEC = 0xA1  # 프리셋 실행 (Value: 1~3)

# ── 최소 높이 설정 ──
CMD_MIN_HEIGHT_SET   = 0xB0  # 최소 높이 저장 (Value: motor_id)
CMD_MIN_HEIGHT_RESET = 0xB1  # 최소 높이 초기화

# ── RF 헤더 ──
RF_HEADER = 0xF0  # RF 24-bit 패킷의 상위 8bit 식별 헤더
RF_RECEIVE_PIN = 27  # RF 수신기 데이터 핀 (GPIO 27, 물리 핀 13)

# ── XBee UART (하단본체 통신) ──
XBEE_PORT = '/dev/serial0'   # RPi GPIO UART (USB 어댑터 사용 시: /dev/ttyUSB0)
XBEE_BAUD = 9600             # XBee 기본 보드레이트

# ── RPi5 모터 및 엔코더 핀 (TB6612FNG) ──
# 모터 A (상단 블라인드: CMD_UPPER)
MOTOR_A_PWMA = 12
MOTOR_A_AIN1 = 23
MOTOR_A_AIN2 = 24
HALL_A1 = 16
HALL_B1 = 20

# 모터 B (하단 프레임: CMD_LOWER_F)
MOTOR_B_PWMB = 13
MOTOR_B_BIN1 = 5
MOTOR_B_BIN2 = 6
HALL_A2 = 21
HALL_B2 = 26

MOTOR_STBY = 25

# ── 엔코더 및 위치 제어 안전 설정 ──
# 엔코더 카운트가 실제 이동 방향과 반대로 변하면 해당 값을 True로 변경한다.
# False 상태에서 모터 이동 시 cnt가 음수로 내려간다면 True로 바꾸면 된다.
MOTOR_A_ENCODER_REVERSED = False
MOTOR_B_ENCODER_REVERSED = False

# 전체 이동에 필요한 실제 엔코더 펄스 수.
# 현재는 기존 코드와 동일하게 100을 사용하며, 실제 측정값으로 보정해야 한다.
MOTOR_A_TOTAL_PULSES = 100
MOTOR_B_TOTAL_PULSES = 100

# 위치 제어 중 엔코더 카운트가 이 시간 동안 변하지 않으면 강제 정지한다.
ENCODER_STALL_TIMEOUT_SEC = 3.0

# 엔코더 값이 변하더라도 이 시간을 초과하여 계속 구동하면 강제 정지한다.
MOTOR_MAX_RUN_TIME_SEC = 20.0

# Command ID → 한글 이름 매핑 (로그 출력용)
CMD_NAMES = {
    CMD_STOP:        "긴급 정지",
    CMD_UPPER:       "상단 블라인드",
    CMD_LOWER_F:     "하단 프레임",
    CMD_LOWER_B:     "하단 블라인드",
    CMD_DIR_UPPER:   "상단 방향제어",
    CMD_DIR_LOWER_F: "하단프레임 방향제어",
    CMD_DIR_LOWER_B: "하단블라인드 방향제어",
    CMD_PRESET_SAVE: "프리셋 저장",
    CMD_PRESET_EXEC: "프리셋 실행",
    CMD_MIN_HEIGHT_SET:   "최소높이 설정",
    CMD_MIN_HEIGHT_RESET: "최소높이 초기화",
}

# 방향 제어 CMD → 위치 제어 CMD 매핑 (정지 시 Notify용)
DIR_TO_POS_CMD = {
    CMD_DIR_UPPER:   CMD_UPPER,
    CMD_DIR_LOWER_F: CMD_LOWER_F,
    CMD_DIR_LOWER_B: CMD_LOWER_B,
}

# ==========================================
# 현재 블라인드 위치 상태 (0~100%)
# ==========================================
# 모터 구동 후 위치를 추적하여 앱에 Notify로 전송할 때 사용한다.
# 향후 엔코더 기반 위치 추정 로직이 구현되면 이 값이 실시간으로 갱신된다.
current_positions = {
    CMD_UPPER:   0,   # 상단 블라인드 현재 위치 (%)
    CMD_LOWER_F: 0,   # 하단 프레임 현재 위치 (%)
    CMD_LOWER_B: 0,   # 하단 블라인드 현재 위치 (%)
}

# ==========================================
# 프리셋 데이터 (최대 3개, JSON 파일로 영속 저장)
# ==========================================
PRESET_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'presets.json')
MAX_PRESETS = 3

# 프리셋 구조: { "1": {"upper": 0, "frame": 50, "blind": 100}, ... }
presets = {}

def load_presets():
    """프리셋 데이터를 JSON 파일에서 불러온다."""
    global presets
    try:
        if os.path.exists(PRESET_FILE):
            with open(PRESET_FILE, 'r') as f:
                presets = json.load(f)
            print(f"📂 프리셋 {len(presets)}개 로드 완료")
        else:
            presets = {}
            print("📂 저장된 프리셋 없음 (새로 생성)")
    except Exception as e:
        print(f"⚠️ 프리셋 로드 실패: {e}")
        presets = {}

def save_presets():
    """프리셋 데이터를 JSON 파일에 저장한다."""
    try:
        with open(PRESET_FILE, 'w') as f:
            json.dump(presets, f, indent=2)
        print(f"💾 프리셋 저장 완료 ({PRESET_FILE})")
    except Exception as e:
        print(f"⚠️ 프리셋 저장 실패: {e}")

# ==========================================
# 최소 높이 설정 (모터별 독립, 0~100%)
# ==========================================
min_heights = {
    CMD_UPPER:   100,  # 상단 블라인드 최소 높이 (기본: 100% = 제한 없음)
    CMD_LOWER_F: 100,  # 하단 프레임
    CMD_LOWER_B: 100,  # 하단 블라인드
}

# ==========================================
# 라즈베리파이 5 모터 제어 클래스 (gpiozero + lgpio)
# ==========================================
class TB6612FNGLinux:
    def __init__(self, pwm_pin, in1_pin, in2_pin):
        self.pwm = PWMOutputDevice(pwm_pin, frequency=1000)
        self.in1 = DigitalOutputDevice(in1_pin)
        self.in2 = DigitalOutputDevice(in2_pin)

    def forward(self, speed):
        self.in1.on()
        self.in2.off()
        self.pwm.value = min(max(speed, 0), 1.0)

    def backward(self, speed):
        self.in1.off()
        self.in2.on()
        self.pwm.value = min(max(speed, 0), 1.0)

    def stop(self):
        self.in1.off()
        self.in2.off()
        self.pwm.value = 0

    def brake(self):
        self.in1.on()
        self.in2.on()
        self.pwm.value = 0

class HallSensorLinux:
    def __init__(self, h_chip, pin_a, pin_b, total_pulses=100, reverse_direction=False):
        self.h_chip = h_chip
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.total_pulses = total_pulses
        self.reverse_direction = reverse_direction
        self.count = 0

        # 풀업 입력 설정
        lgpio.gpio_claim_input(self.h_chip, self.pin_a, lgpio.SET_PULL_UP)
        lgpio.gpio_claim_input(self.h_chip, self.pin_b, lgpio.SET_PULL_UP)
        # ISR 등록 (A상승 에지)
        # callback 반환 객체를 멤버 변수로 보관하여 콜백 참조가 유지되도록 한다.
        self.cb = lgpio.callback(
            self.h_chip,
            self.pin_a,
            lgpio.RISING_EDGE,
            self._isr
        )

    def _isr(self, chip, gpio, level, tick):
        b_val = lgpio.gpio_read(self.h_chip, self.pin_b)

        # A상 상승 에지에서 B상 값을 읽어 회전 방향을 판별한다.
        # 기존 코드의 0~total_pulses 제한을 제거하여, 방향이 반대로 잡힌 경우
        # count가 음수로 내려가는 모습을 로그에서 확인할 수 있게 한다.
        delta = 1 if b_val else -1

        if self.reverse_direction:
            delta *= -1

        self.count += delta

    @property
    def pos(self):
        if self.total_pulses == 0: return 0
        # 원시 count는 진단을 위해 음수/초과값을 허용하지만,
        # 앱과 PID에서 사용하는 위치값은 기존과 동일하게 0~100%로 제한한다.
        return max(0, min(100, int(self.count / self.total_pulses * 100)))

class PIDController:
    def __init__(self, kp=0.05, ki=0.0, kd=0.01):
        self.kp = kp; self.ki = ki; self.kd = kd
        self.err_sum = 0; self.prev_err = 0; self.prev_t = time.time()
    def reset(self):
        self.err_sum = 0; self.prev_err = 0; self.prev_t = time.time()
    def compute(self, curr, tgt):
        err = tgt - curr
        if abs(err) <= 2: return 0, True
        now = time.time()
        dt = max(now - self.prev_t, 0.01)
        self.prev_t = now
        self.err_sum = max(-50, min(50, self.err_sum + err * dt))
        d_err = (err - self.prev_err) / dt
        self.prev_err = err
        out = (self.kp * err) + (self.ki * self.err_sum) + (self.kd * d_err)
        return max(-1.0, min(1.0, out)), False

class MotorNode:
    def __init__(self, name, mot, hal, pid):
        self.name = name; self.mot = mot; self.hal = hal; self.pid = pid
        self.mode = 0  # 0: IDLE, 1: POS, 2: DIR
        self.tgt = 0; self.dir = 0
        self._log_cnt = 0

        # 위치 제어 안전정지용 상태값
        self.started_at = 0.0
        self.last_encoder_count = self.hal.count
        self.last_encoder_change_at = time.time()

    def start_position_control(self, target):
        """목표 위치 제어를 시작하고 안전정지 타이머를 초기화한다."""
        now = time.time()
        self.tgt = target
        self.pid.reset()
        self.started_at = now
        self.last_encoder_count = self.hal.count
        self.last_encoder_change_at = now
        self.mode = 1

    def stop_position_control(self):
        """위치 제어를 중지하고 모터를 정지한다."""
        self.mot.stop()
        self.mode = 0

    def update(self):
        if self.mode == 1:
            now = time.time()
            cur = self.hal.pos
            out, done = self.pid.compute(cur, self.tgt)

            # 엔코더 원시 카운트가 변했으면 마지막 변화 시간을 갱신한다.
            if self.hal.count != self.last_encoder_count:
                self.last_encoder_count = self.hal.count
                self.last_encoder_change_at = now

            # 디버그: 50회(≈0.5s)마다 엔코더 실측값 로그
            self._log_cnt += 1
            if self._log_cnt >= 50:
                self._log_cnt = 0
                print(f"   🔍 [{self.name}] encoder={cur}% tgt={self.tgt}% pid_out={out:.3f} cnt={self.hal.count}/{self.hal.total_pulses}")

            if done:
                print(f"   ✅ [{self.name}] 도착! encoder={cur}% tgt={self.tgt}%")
                self.stop_position_control()
                return

            # 엔코더 값이 일정 시간 동안 전혀 변하지 않으면 강제 정지한다.
            if now - self.last_encoder_change_at >= ENCODER_STALL_TIMEOUT_SEC:
                print(
                    f"   ⚠️ [{self.name}] 엔코더 펄스 미감지 "
                    f"({ENCODER_STALL_TIMEOUT_SEC:.1f}초) – 강제 정지 "
                    f"cnt={self.hal.count}, tgt={self.tgt}%"
                )
                self.stop_position_control()
                return

            # 엔코더가 변하더라도 최대 구동시간을 넘으면 강제 정지한다.
            if now - self.started_at >= MOTOR_MAX_RUN_TIME_SEC:
                print(
                    f"   ⚠️ [{self.name}] 최대 구동시간 초과 "
                    f"({MOTOR_MAX_RUN_TIME_SEC:.1f}초) – 강제 정지 "
                    f"encoder={cur}%, tgt={self.tgt}%"
                )
                self.stop_position_control()
                return

            if out > 0: self.mot.forward(out)
            else: self.mot.backward(-out)
        elif self.mode == 2:
            if self.dir == 1: self.mot.forward(1.0)
            elif self.dir == 2: self.mot.backward(1.0)
            else: self.mot.stop()
        else:
            self.mot.stop()

# ── 글로벌 노드 및 핸들 ──
lgpio_handle = None
motor_nodes = {}
stby_pin = None

def motor_loop():
    while rf_running:
        for node in motor_nodes.values(): node.update()
        time.sleep(0.01)

# ==========================================
# 전역 상태 변수
# ==========================================
# BLE Peripheral 전역 참조 (signal 핸들러에서 접근하기 위함)
ble_app = None

# RF 수신 스레드 종료 플래그
rf_running = True

# XBee 시리얼 객체 (하단본체 통신)
xb_ser = None


def on_connect(device):
    """
    BLE 연결 성공 콜백.

    스마트폰 앱이 이 라즈베리파이 BLE 서버에 연결되었을 때 호출된다.
    bluezero 라이브러리가 연결 이벤트 발생 시 자동으로 호출한다.

    Args:
        device: 연결된 BLE Central 장치 객체.
                device.address로 스마트폰의 MAC 주소를 확인할 수 있다.
    """
    print("✅ 스마트폰 연결됨: " + str(device.address))


def on_disconnect(adapter_address, device_address):
    """
    BLE 연결 해제 콜백.

    스마트폰 앱과의 BLE 연결이 끊어졌을 때 호출된다.
    사용자가 앱을 종료하거나, 블루투스 범위를 벗어나면 발생한다.

    Args:
        adapter_address: 라즈베리파이 BLE 어댑터의 MAC 주소.
        device_address:  연결이 끊어진 스마트폰의 MAC 주소.
    """
    print("❌ 연결 끊김: " + str(device_address))


def send_status_notify(cmd_id, value):
    """
    Status TX Characteristic을 통해 앱에 현재 상태를 Notify로 전송한다.

    모터 구동 완료 후, 또는 센서 데이터 갱신 시 이 함수를 호출하여
    앱에 실시간 피드백을 보낸다.

    전송 패킷 구조 (Command RX와 동일한 2바이트 형식):
        Byte 0: Command ID (어떤 모터/센서의 데이터인지)
        Byte 1: Value (현재 위치 0~100% 또는 센서 값)

    Args:
        cmd_id: 상태를 보고할 대상의 Command ID (CMD_UPPER, CMD_LOWER_F 등)
        value:  현재 위치 (0~100) 또는 센서 측정값
    """
    global ble_app
    if ble_app is None:
        return

    try:
        # TODO: bluezero에서 characteristic set_value 방식으로 수정 필요
        # ble_app.update_value(srv_id=1, chr_id=2, value=[cmd_id, value])
        print(f"   📤 상태 Notify 시뮬레이션: [0x{cmd_id:02x}, 0x{value:02x}]")
    except Exception as e:
        print(f"   ⚠️ Notify 전송 실패: {e}")


# ==========================================
# XBee 포워딩 (UART → 하단본체)
# ==========================================

def xbee_init():
    """
    XBee UART 초기화.

    RPi5의 GPIO UART(/dev/serial0) 또는 USB-시리얼 어댑터(/dev/ttyUSB0)을 통해
    XBee S2C와 Transparent 모드로 통신한다.
    """
    global xb_ser
    try:
        xb_ser = serial.Serial(XBEE_PORT, XBEE_BAUD, timeout=0.1)
        print(f"📡 XBee UART 초기화: {XBEE_PORT} @ {XBEE_BAUD}bps")
    except Exception as e:
        print(f"⚠️ XBee 초기화 실패: {e}")
        print("   pip install pyserial")


def xbee_fwd(cmd_id, val):
    """
    하단본체로 2-Byte 명령 전달.

    Args:
        cmd_id: Command ID
        val:    Value
    """
    if xb_ser and xb_ser.is_open:
        xb_ser.write(bytes([cmd_id, val]))
        print(f"   📤 XBee 포워딩: [0x{cmd_id:02x}, 0x{val:02x}]")


def xbee_recv_loop():
    """
    하단본체 상태 보고 수신 스레드.

    하단본체가 [0xE0, POSITION] 패킷을 보내면
    current_positions[CMD_LOWER_B]를 갱신한다.
    """
    import time as _t
    while rf_running:
        try:
            if xb_ser and xb_ser.is_open and xb_ser.in_waiting >= 2:
                b = xb_ser.read(2)
                if len(b) == 2 and b[0] == 0xE0:
                    current_positions[CMD_LOWER_B] = b[1]
                    print(f"   📥 XBee 하단 상태: {b[1]}%")
        except Exception:
            pass
        _t.sleep(0.01)


# ==========================================
# 통합 명령 핸들러 (BLE + RF 공용)
# ==========================================

def handle_command(cmd_id, val, source="BLE"):
    """
    통합 명령 처리 함수. BLE on_write()와 RF 수신 루프 모두 이 함수를 호출한다.

    Args:
        cmd_id: Command ID (0x00~0xB1)
        val:    Value (0~100, 방향코드, 프리셋번호 등)
        source: 명령 출처 문자열 ("BLE" 또는 "RF", 로그 구분용)
    """
    cmd_name = CMD_NAMES.get(cmd_id, f"0x{cmd_id:02x}")
    print(f"📩 [{source}] 수신: [0x{cmd_id:02x}, 0x{val:02x}] ({cmd_name}, val={val})")

    # ── 긴급 정지 (0x00) → 전체 브로드캐스트 ──
    if cmd_id == CMD_STOP:
        print("   🚨 긴급 정지 실행! 모든 모터 즉시 멈춤")
        xbee_fwd(CMD_STOP, 0x00)                     # 하단본체에도 전달
        for node in motor_nodes.values():
            node.mode = 0
            node.mot.stop()
        send_status_notify(CMD_STOP, 0x00)
        return

    # ── 목표 위치 제어 (0x01~0x03, 앱 + 리모컨 짧게 누르기) ──
    if cmd_id in (CMD_UPPER, CMD_LOWER_F, CMD_LOWER_B):
        clamped_val = max(0, min(100, val))

        # 최소 높이 제한 적용
        max_allowed = min_heights.get(cmd_id, 100)
        if clamped_val > max_allowed:
            print(f"   📏 최소높이 제한 적용: {clamped_val}% → {max_allowed}%")
            clamped_val = max_allowed

        if clamped_val != val:
            print(f"   ⚠️ 값 보정: {val} → {clamped_val}")

        motor_name = CMD_NAMES[cmd_id]
        print(f"   ⚙️ {motor_name} 이동 → {clamped_val}%")
        if cmd_id in (CMD_UPPER, CMD_LOWER_F):
            if cmd_id in motor_nodes:
                # 목표 위치와 PID를 설정하고, 엔코더 무변화/최대 구동시간
                # 감시를 위한 타이머를 함께 초기화한다.
                motor_nodes[cmd_id].start_position_control(clamped_val)
        else:
            xbee_fwd(cmd_id, clamped_val)            # BLIND → 하단본체

        current_positions[cmd_id] = clamped_val
        send_status_notify(cmd_id, clamped_val)
        return

    # ── 방향 제어 (0x11/0x21/0x31, 리모컨 길게 누르기) ──
    if cmd_id in (CMD_DIR_UPPER, CMD_DIR_LOWER_F, CMD_DIR_LOWER_B):
        pos_cmd = DIR_TO_POS_CMD[cmd_id]
        motor_name = CMD_NAMES.get(cmd_id, "?")

        if val == DIR_UP:
            print(f"   ⬆️ {motor_name}: 올림 시작")
            if cmd_id in (CMD_DIR_UPPER, CMD_DIR_LOWER_F):
                if pos_cmd in motor_nodes:
                    motor_nodes[pos_cmd].dir = 1
                    motor_nodes[pos_cmd].mode = 2
            else:
                xbee_fwd(cmd_id, val)
        elif val == DIR_DOWN:
            print(f"   ⬇️ {motor_name}: 내림 시작")
            if cmd_id in (CMD_DIR_UPPER, CMD_DIR_LOWER_F):
                if pos_cmd in motor_nodes:
                    motor_nodes[pos_cmd].dir = 2
                    motor_nodes[pos_cmd].mode = 2
            else:
                xbee_fwd(cmd_id, val)
        elif val == DIR_STOP:
            print(f"   ⏹️ {motor_name}: 정지")
            if cmd_id in (CMD_DIR_UPPER, CMD_DIR_LOWER_F):
                if pos_cmd in motor_nodes:
                    motor_nodes[pos_cmd].mode = 0
                    motor_nodes[pos_cmd].mot.stop()
            else:
                xbee_fwd(cmd_id, val)
            send_status_notify(pos_cmd, current_positions.get(pos_cmd, 0))
        else:
            print(f"   ⚠️ 알 수 없는 방향 값: 0x{val:02x}")
        return

    # ── 프리셋 저장 (0xA0) ──
    if cmd_id == CMD_PRESET_SAVE:
        preset_num = str(val)
        if val < 1 or val > MAX_PRESETS:
            print(f"   ⚠️ 프리셋 번호 범위 초과: {val} (1~{MAX_PRESETS})")
            return

        presets[preset_num] = {
            "upper": current_positions.get(CMD_UPPER, 0),
            "frame": current_positions.get(CMD_LOWER_F, 0),
            "blind": current_positions.get(CMD_LOWER_B, 0),
        }
        save_presets()
        print(f"   💾 프리셋 {val} 저장: {presets[preset_num]}")
        return

    # ── 프리셋 실행 (0xA1) ──
    if cmd_id == CMD_PRESET_EXEC:
        preset_num = str(val)
        if preset_num not in presets:
            print(f"   ⚠️ 프리셋 {val}번이 저장되어 있지 않음")
            return

        p = presets[preset_num]
        print(f"   ▶️ 프리셋 {val} 실행: {p}")

        # 3개 모터를 순차 구동 (50ms 간격과 동일한 효과)
        handle_command(CMD_UPPER, p.get("upper", 0), source)
        handle_command(CMD_LOWER_F, p.get("frame", 0), source)
        handle_command(CMD_LOWER_B, p.get("blind", 0), source)
        return

    # ── 최소 높이 설정 (0xB0) ──
    if cmd_id == CMD_MIN_HEIGHT_SET:
        motor_id = val
        if motor_id in current_positions:
            pos = current_positions[motor_id]
            min_heights[motor_id] = pos
            if motor_id not in (CMD_UPPER, CMD_LOWER_F):
                xbee_fwd(cmd_id, motor_id)           # 하단본체에 전달
            motor_name = CMD_NAMES.get(motor_id, "?")
            print(f"   📏 {motor_name} 최소 높이 설정: {pos}%")
        else:
            print(f"   ⚠️ 알 수 없는 모터 ID: 0x{motor_id:02x}")
        return

    # ── 최소 높이 초기화 (0xB1) ──
    if cmd_id == CMD_MIN_HEIGHT_RESET:
        motor_id = val
        if motor_id in min_heights:
            min_heights[motor_id] = 100
            if motor_id not in (CMD_UPPER, CMD_LOWER_F):
                xbee_fwd(cmd_id, motor_id)           # 하단본체에 전달
            motor_name = CMD_NAMES.get(motor_id, "?")
            print(f"   📏 {motor_name} 최소 높이 초기화 (제한 해제)")
        else:
            print(f"   ⚠️ 알 수 없는 모터 ID: 0x{motor_id:02x}")
        return

    # ── 알 수 없는 Command ID ──
    print(f"   ⚠️ 알 수 없는 Command ID: 0x{cmd_id:02x}. 패킷 무시.")


def on_write(value, options):
    """
    BLE Write 콜백. 앱에서 보낸 2-Byte 패킷을 파싱하여 handle_command()에 전달.

    Args:
        value:   앱에서 전송한 바이트 배열 (dbus-python Array → bytes 변환 필요).
        options: D-Bus Write 옵션 딕셔너리 (현재 미사용).
    """
    try:
        data = bytes(value)
        if len(data) != 2:
            print(f"⚠️ 패킷 길이 오류: {len(data)}바이트 수신 (2바이트 필요). 폐기.")
            return
        handle_command(data[0], data[1], source="BLE")
    except Exception as e:
        print(f"⚠️ BLE 데이터 해석 오류: {e}")


# ==========================================
# RF433 수신 스레드
# ==========================================

def rf_receive_loop():
    """
    RF433 수신기에서 24-bit 데이터를 수신하여 파싱하는 루프.
    별도 스레드에서 실행된다.

    수신 데이터 구조: [0xF0][CMD_ID][VALUE] (24-bit)
      - 상위 8bit가 0xF0이면 유효한 리모컨 패킷으로 인식
      - 하위 16bit를 [CMD_ID, VALUE]로 분리하여 handle_command() 호출

    의존성:
      rpi-rf 라이브러리 (pip install rpi-rf)
      ⚠️ RPi5에서 rpi-rf가 내부적으로 RPi.GPIO를 사용하는 경우,
         rpi-lgpio 호환 패키지를 함께 설치해야 할 수 있음:
         pip install rpi-lgpio
    """
    global rf_running

    try:
        from rpi_rf import RFDevice
    except ImportError:
        print("⚠️ rpi-rf 라이브러리 미설치. RF 수신 비활성화.")
        print("   설치: pip install rpi-rf rpi-lgpio")
        return

    rfdevice = None
    try:
        rfdevice = RFDevice(RF_RECEIVE_PIN)
        rfdevice.enable_rx()
        print(f"📻 RF433 수신기 활성화 (GPIO {RF_RECEIVE_PIN})")

        last_timestamp = 0

        while rf_running:
            if rfdevice.rx_code_timestamp != last_timestamp:
                last_timestamp = rfdevice.rx_code_timestamp
                code = rfdevice.rx_code

                # 24-bit 코드 수신 로그
                print(f"📻 RF 수신: 0x{code:06X} (proto={rfdevice.rx_proto}, "
                      f"pulse={rfdevice.rx_pulselength})")

                # 유효성 검증: 상위 8bit가 0xF0인지 확인
                header = (code >> 16) & 0xFF
                if header != RF_HEADER:
                    print(f"   ⚠️ RF 헤더 불일치 (0x{header:02X} != 0xF0). 무시.")
                    continue

                # 하위 16bit에서 CMD_ID, VALUE 추출
                cmd_id = (code >> 8) & 0xFF
                val = code & 0xFF

                handle_command(cmd_id, val, source="RF")

            # CPU 부하 방지 (10ms 간격 폴링)
            import time
            time.sleep(0.01)

    except Exception as e:
        print(f"⚠️ RF 수신 오류: {e}")
    finally:
        if rfdevice:
            rfdevice.cleanup()
            print("📻 RF 수신기 정리 완료")


def on_status_read():
    """
    Status TX Characteristic의 Read 콜백.

    앱이 Status TX Characteristic을 Read 요청할 때 호출된다.
    현재 블라인드 위치 상태를 3바이트로 반환한다.

    반환 형식: [상단 위치(%), 하단 프레임 위치(%), 하단 블라인드 위치(%)]
    """
    return [
        current_positions.get(CMD_UPPER, 0),
        current_positions.get(CMD_LOWER_F, 0),
        current_positions.get(CMD_LOWER_B, 0),
    ]


def signal_handler(sig, frame):
    """
    Graceful Shutdown 핸들러.

    Ctrl+C (SIGINT) 또는 SIGTERM 수신 시 호출되어,
    BLE 리소스를 정리하고 프로그램을 안전하게 종료한다.

    이 핸들러가 없으면:
      - BLE 광고가 남아있어 다음 실행 시 충돌할 수 있다.
      - D-Bus 리소스가 해제되지 않아 'Address already in use' 에러가 발생할 수 있다.
    """
    global rf_running
    print("\n🛑 종료 신호 수신 – 서버 정리 중...")

    # RF 수신 스레드 종료
    rf_running = False

    # BLE Peripheral 리소스 정리
    # bluezero의 publish()가 내부적으로 GLib MainLoop를 사용하므로,
    # 별도의 정리 메서드가 제공되지 않을 수 있다.
    # 이 경우 프로세스 종료 시 OS가 D-Bus 연결을 자동 해제한다.

    print("   ✅ 서버 종료 완료.")
    sys.exit(0)


def main():
    """
    BLE Peripheral 서버 초기화 및 실행.

    실행 순서:
        1. 시스템의 BLE 어댑터(hci0) 주소를 자동 감지
        2. Peripheral 객체를 생성하여 BLE GATT 서버를 구성
        3. GATT 서비스와 Characteristic을 등록
           - chr_id=1 (ffe1): Command RX – 앱에서 제어 명령 수신 (Write)
           - chr_id=2 (ffe2): Status TX  – 앱에 상태 데이터 전송 (Notify + Read)
        4. 광고(Advertising)를 시작하여 스마트폰에서 검색 가능하게 설정
        5. 이벤트 루프에 진입하여 연결/데이터 수신을 대기

    GATT 구조:
        Service (SERVICE_UUID: 0000ffe0-...)
        ├── Characteristic 1 (CMD_CHAR_UUID: 0000ffe1-...)
        │   ├── Flags: write, write-without-response
        │   └── Write Callback: on_write()
        └── Characteristic 2 (STATUS_CHAR_UUID: 0000ffe2-...)  ← [NEW]
            ├── Flags: notify, read
            └── Read Callback: on_status_read()

    참고:
        - local_name='IoTSBC'는 스마트폰 BLE 스캔 시 표시되는 장치 이름이다.
        - appearance=0은 "Unknown" 장치 유형을 의미한다.
        - publish() 호출 후 블로킹 이벤트 루프에 진입하며,
          프로그램이 종료될 때까지 BLE 연결을 계속 수신한다.
    """
    global ble_app

    # ── 0단계: 초기화 ──
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    load_presets()

    # RPi5 로컬 모터 초기화
    global lgpio_handle, stby_pin
    try:
        lgpio_handle = lgpio.gpiochip_open(0)
        stby_pin = DigitalOutputDevice(MOTOR_STBY, initial_value=True)
        # 상단 블라인드
        motA = TB6612FNGLinux(MOTOR_A_PWMA, MOTOR_A_AIN1, MOTOR_A_AIN2)
        halA = HallSensorLinux(
            lgpio_handle,
            HALL_A1,
            HALL_B1,
            MOTOR_A_TOTAL_PULSES,
            reverse_direction=MOTOR_A_ENCODER_REVERSED
        )
        motor_nodes[CMD_UPPER] = MotorNode("UPPER", motA, halA, PIDController())
        # 하단 프레임
        motB = TB6612FNGLinux(MOTOR_B_PWMB, MOTOR_B_BIN1, MOTOR_B_BIN2)
        halB = HallSensorLinux(
            lgpio_handle,
            HALL_A2,
            HALL_B2,
            MOTOR_B_TOTAL_PULSES,
            reverse_direction=MOTOR_B_ENCODER_REVERSED
        )
        motor_nodes[CMD_LOWER_F] = MotorNode("FRAME", motB, halB, PIDController())
        threading.Thread(target=motor_loop, daemon=True).start()
        print(f"   ⚙️ RPi5 로컬 모터 스레드 시작 (CMD:0x01, 0x02)")
    except Exception as e:
        print(f"   ⚠️ RPi5 모터 초기화 실패: {e}")

    # XBee UART 초기화 (하단본체 통신)
    xbee_init()
    threading.Thread(target=xbee_recv_loop, daemon=True).start()

    # ── 1단계: BLE 어댑터 확인 ──
    # 라즈베리파이에 장착된 BLE 어댑터(hci0) 목록에서 첫 번째 어댑터의
    # MAC 주소를 가져온다.
    # [v2 개선] 어댑터가 없을 경우 명확한 에러 메시지 출력 후 종료.
    # RPi5에서 Wi-Fi를 dtoverlay로 비활성화하면 블루투스도 함께 꺼질 수 있으므로,
    # 어댑터 존재 여부를 반드시 확인해야 한다.
    adapters = list(adapter.Adapter.available())
    if not adapters:
        print("❌ BLE 어댑터를 찾을 수 없습니다!")
        print("   확인 사항:")
        print("   1. bluetoothctl show  → 어댑터 정보 확인")
        print("   2. hciconfig -a       → hci0 상태 확인")
        print("   3. sudo systemctl status bluetooth → 서비스 상태 확인")
        print("   4. /boot/config.txt에서 dtoverlay=disable-wifi 사용 시")
        print("      블루투스도 함께 비활성화될 수 있음")
        sys.exit(1)

    adapter_addr = adapters[0].address
    print(f"📡 라즈베리파이5 BLE 시작 (MAC: {adapter_addr})")
    print(f"   프로토콜: 2-Byte 바이너리 패킷 [CMD_ID, VALUE]")

    # ── 2단계: BLE Peripheral(GATT 서버) 객체 생성 ──
    # local_name : 스마트폰 BLE 스캔 목록에 표시될 장치 이름
    #              앱(bleManager.ts)은 이 이름("IoTSBC")으로 기기를 식별한다.
    # appearance : BLE 장치 외형 코드 (0 = Unknown/Generic)
    ble_app = peripheral.Peripheral(adapter_addr, local_name='IoTSBC', appearance=0)

    # ── 3단계: GATT 서비스 및 Characteristic 등록 ──

    # 서비스(Service) 등록:
    #   srv_id=1  : 내부 서비스 식별 번호 (bluezero 내부에서 사용)
    #   uuid      : 앱과 약속된 서비스 UUID
    #   primary   : True → 이 서비스가 기본(Primary) 서비스임을 명시
    ble_app.add_service(srv_id=1, uuid=SERVICE_UUID, primary=True)

    # Characteristic 1: Command RX (ffe1) – 앱 → RPi 제어 명령 수신
    #   srv_id=1, chr_id=1 : 서비스 1번 아래의 1번 특성
    #   uuid               : 앱과 약속된 Command RX Characteristic UUID
    #   value=[]           : 초기값 (빈 바이트 배열)
    #   notifying=False    : 이 Characteristic은 Notify를 사용하지 않음
    #   flags              : 'write' (응답 있는 쓰기) + 'write-without-response' (응답 없는 쓰기)
    #                        두 모드 모두 허용하여 앱 호환성을 높임
    #   write_callback     : 앱이 이 Characteristic에 Write할 때 호출될 콜백 함수
    ble_app.add_characteristic(srv_id=1, chr_id=1, uuid=CMD_CHAR_UUID,
                               value=[], notifying=False,
                               flags=['write', 'write-without-response'],
                               write_callback=on_write)

    # [NEW] Characteristic 2: Status TX (ffe2) – RPi → 앱 상태/센서 피드백
    #   1차 캡스톤 최종보고서 2-3-2절에 정의된 규격.
    #   모터 구동 완료 후 현재 위치, 또는 센서 데이터(온도 등)를
    #   앱에 Notify로 전송하는 용도로 사용한다.
    #
    #   srv_id=1, chr_id=2 : 서비스 1번 아래의 2번 특성
    #   uuid               : Status TX Characteristic UUID (ffe2)
    #   value=[0, 0, 0]    : 초기값 [상단 위치, 하단프레임 위치, 하단블라인드 위치]
    #   notifying=False    : 초기에는 Notify 비활성 (앱이 구독 시 자동 활성화)
    #   flags              : 'notify' (서버→클라이언트 알림) + 'read' (클라이언트 폴링)
    #   read_callback      : 앱이 이 Characteristic을 Read할 때 호출될 콜백 함수
    ble_app.add_characteristic(srv_id=1, chr_id=2, uuid=STATUS_CHAR_UUID,
                               value=[0, 0, 0], notifying=False,
                               flags=['notify', 'read'],
                               read_callback=on_status_read)

    # ── 4단계: 연결/해제 콜백 등록 및 BLE 광고(Advertising) 시작 ──
    # 스마트폰이 연결/해제될 때 호출될 콜백 함수를 등록한다.
    ble_app.on_connect = on_connect
    ble_app.on_disconnect = on_disconnect

    # publish()를 호출하면:
    #   1) BLE 광고(Advertising)가 시작되어 스마트폰에서 검색 가능해진다.
    #   2) D-Bus 이벤트 루프에 진입하여 연결 요청과 데이터 수신을 대기한다.
    #   3) 이 호출은 블로킹(blocking)이므로, 이후 코드는 실행되지 않는다.
    #   4) 종료 시 signal_handler가 호출되어 안전하게 정리된다.
    # ── 5단계: RF433 수신 스레드 시작 ──
    # BLE 이벤트 루프와 별도로 RF 수신을 처리하기 위해
    # 데몬 스레드로 실행한다. (메인 종료 시 자동 종료)
    rf_thread = threading.Thread(target=rf_receive_loop, daemon=True)
    rf_thread.start()

    # ── 6단계: BLE 광고 시작 및 이벤트 루프 진입 ──
    print("🚀 앱/리모컨 연결 대기 중... (Ctrl+C로 종료)")
    print(f"   GATT Service: {SERVICE_UUID}")
    print(f"   ├─ Command RX (ffe1): Write")
    print(f"   └─ Status TX  (ffe2): Notify + Read")
    print(f"   📻 RF433 수신: GPIO {RF_RECEIVE_PIN} (헤더 0xF0)")
    print(f"   📡 XBee UART: {XBEE_PORT} @ {XBEE_BAUD}bps")
    ble_app.publish()


# ── 프로그램 진입점 ──
# 이 스크립트가 직접 실행될 때만 main()을 호출한다.
# 다른 모듈에서 import할 경우에는 main()이 자동 실행되지 않는다.
if __name__ == '__main__':
    main()