# -*- coding: utf-8 -*-
"""
하단 본체 Arduino Nano ESP32 – 하단 블라인드 전용
================================================

[0804 19시 10분 업데이트]

구조
----
- 상단 본체(Raspberry Pi)
  · 상단 블라인드 제어
  · 하단 본체 위치 제어
  · RF 리모컨/BLE 명령 수신
  · 하단 블라인드 명령만 XBee로 전달

- 하단 본체(Arduino Nano ESP32)
  · XBee 명령 수신
  · 하단 블라인드 모터 1개만 제어
  · 실제 엔코더 위치를 XBee로 상단 본체에 보고

실제 리모컨 코드와 대응
----------------------
키 3 짧게      → [0x03, 0x00] : 하단 블라인드 완전 올림
키 6 짧게      → [0x03, 0x64] : 하단 블라인드 완전 내림
키 3 길게      → [0x31, 0x01] : 올림 시작
키 3 뗌        → [0x31, 0x00] : 정지
키 6 길게      → [0x31, 0x02] : 내림 시작
키 6 뗌        → [0x31, 0x00] : 정지
* 전체 올림    → 상단 본체가 [0x31, ...]만 하단으로 전달
# 전체 내림    → 상단 본체가 [0x31, ...]만 하단으로 전달
0             → [0x00, 0x00] : 긴급 정지
3/6 더블탭    → [0xB0, 0x03] : 현재 위치를 하단 제한 위치로 저장

프리셋
------
리모컨의 [0xA0, 번호], [0xA1, 번호]는 상단 본체가 처리한다.
프리셋 실행 시 상단 본체가 최종 하단 목표를 [0x03, 위치]로 보내므로,
하단 Arduino는 프리셋 자체를 저장하거나 해석하지 않는다.

제어 방식
---------
- 상단 본체와 동일하게 A상 펄스만 사용
- 위치 증가/감소 방향은 모터 명령 방향으로 결정
- 목표 위치 오차 ±2% 이내이면 정지
- 모터 구동 중 2초간 count가 변하지 않으면 안전 정지
- 목표값을 현재 위치로 즉시 저장하지 않음
- 실제 엔코더 위치만 [0xE0, 위치]로 상단 본체에 송신

핀 연결
-------
Nano D0/RX  (GPIO44) ← XBee DOUT
Nano D1/TX  (GPIO43) → XBee DIN
Nano D2     (GPIO5)  ← 엔코더 A
Nano D3     (GPIO6)  ← 엔코더 B (의도적으로 미사용)
Nano D5     (GPIO8)  → TB6612FNG PWMA
Nano D6     (GPIO9)  → TB6612FNG AIN1
Nano D7     (GPIO10) → TB6612FNG AIN2
Nano D8     (GPIO17) → TB6612FNG STBY

주의
----
- Nano ESP32, XBee, TB6612FNG의 GND는 반드시 공통 연결
- TOTAL_PULSES는 하단 블라인드 전체 구간 실측값으로 교체
- 테스트 완료 후 보드에는 main.py 이름으로 저장
"""

from machine import Pin, PWM, UART
from time import sleep_ms, ticks_ms, ticks_diff

try:
    import ujson as json
except ImportError:
    import json


# ============================================================
# 1. 하단 블라인드에서 사용하는 명령만 정의
# ============================================================

CMD_STOP = 0x00

CMD_LOWER_BLIND_POSITION = 0x03
CMD_LOWER_BLIND_DIRECTION = 0x31

CMD_MIN_HEIGHT_SET = 0xB0
CMD_MIN_HEIGHT_RESET = 0xB1

DIR_STOP = 0x00
DIR_UP = 0x01
DIR_DOWN = 0x02

STATUS_LOWER_BLIND_POSITION = 0xE0
LOWER_BLIND_MOTOR_ID = 0x03


# ============================================================
# 2. 핀 설정
# ============================================================

# XBee UART
XBEE_TX_GPIO = 43       # Nano D1/TX → XBee DIN
XBEE_RX_GPIO = 44       # Nano D0/RX ← XBee DOUT
XBEE_BAUD = 9600

# TB6612FNG
MOTOR_PWM_GPIO = 8      # Nano D5
MOTOR_IN1_GPIO = 9      # Nano D6
MOTOR_IN2_GPIO = 10     # Nano D7
MOTOR_STBY_GPIO = 17    # Nano D8

# 엔코더
ENCODER_A_GPIO = 5      # Nano D2
ENCODER_B_GPIO = 6      # Nano D3, 의도적으로 미사용


# ============================================================
# 3. 조정값
# ============================================================

# 하단 블라인드 0% → 100% 전체 구간의 A상 카운트
# 상승·하강 에지를 모두 세므로 실제 측정값으로 교체해야 함
TOTAL_PULSES = 10000

POSITION_DEADBAND = 2
STALL_TIMEOUT_MS = 2000

PWM_FREQUENCY = 1000
FULL_PWM = 65535

STATUS_CHANGE_INTERVAL_MS = 250
STATUS_HEARTBEAT_MS = 1000
STATE_SAVE_INTERVAL_MS = 5000

STATE_FILE = "lower_blind_state.json"


# ============================================================
# 4. 모터 드라이버
# ============================================================

class TB6612FNG:
    def __init__(self, pwm_gpio, in1_gpio, in2_gpio, stby_gpio):
        self.in1 = Pin(in1_gpio, Pin.OUT, value=0)
        self.in2 = Pin(in2_gpio, Pin.OUT, value=0)
        self.stby = Pin(stby_gpio, Pin.OUT, value=0)

        self.pwm = PWM(Pin(pwm_gpio))
        self.pwm.freq(PWM_FREQUENCY)
        self.pwm.duty_u16(0)

        self.stop()
        self.stby.value(1)

    def forward(self, duty=FULL_PWM):
        """상단 본체와 동일: count가 증가하는 방향."""
        self.stby.value(1)
        self.in1.value(1)
        self.in2.value(0)
        self.pwm.duty_u16(max(0, min(FULL_PWM, int(duty))))

    def backward(self, duty=FULL_PWM):
        """상단 본체와 동일: count가 감소하는 방향."""
        self.stby.value(1)
        self.in1.value(0)
        self.in2.value(1)
        self.pwm.duty_u16(max(0, min(FULL_PWM, int(duty))))

    def stop(self):
        self.pwm.duty_u16(0)
        self.in1.value(0)
        self.in2.value(0)

    def disable(self):
        self.stop()
        self.stby.value(0)


# ============================================================
# 5. A상 전용 엔코더
# ============================================================

class EncoderAOnly:
    def __init__(self, pin_gpio, total_pulses):
        self.total_pulses = max(1, int(total_pulses))
        self.count = 0

        # +1: count 증가, -1: count 감소
        self.direction = 1

        self.pin_a = Pin(pin_gpio, Pin.IN, Pin.PULL_UP)
        self.pin_a.irq(
            trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING,
            handler=self._on_edge,
        )

    def _on_edge(self, _pin):
        new_count = self.count + self.direction

        if new_count < 0:
            new_count = 0
        elif new_count > self.total_pulses:
            new_count = self.total_pulses

        self.count = new_count

    @property
    def position(self):
        return max(
            0,
            min(100, int(self.count * 100 / self.total_pulses)),
        )

    def restore_position(self, position):
        position = max(0, min(100, int(position)))
        self.count = int(position * self.total_pulses / 100)


# ============================================================
# 6. 상단 본체와 동일한 모터 제어 구조
# ============================================================

MODE_IDLE = 0
MODE_POSITION = 1
MODE_DIRECTION = 2


class LowerBlindMotor:
    def __init__(self, driver, encoder):
        self.driver = driver
        self.encoder = encoder

        self.mode = MODE_IDLE
        self.target = 0
        self.direction_command = DIR_STOP

        self.last_count = encoder.count
        self.last_count_time = ticks_ms()

    def _reset_stall_timer(self):
        self.last_count = self.encoder.count
        self.last_count_time = ticks_ms()

    def _check_stall(self):
        now = ticks_ms()
        current_count = self.encoder.count

        if current_count != self.last_count:
            self.last_count = current_count
            self.last_count_time = now
            return False

        if ticks_diff(now, self.last_count_time) >= STALL_TIMEOUT_MS:
            print(
                "[안전정지] 엔코더 count가",
                STALL_TIMEOUT_MS,
                "ms 동안 변하지 않음:",
                current_count,
            )
            self.stop()
            return True

        return False

    def move_to(self, target):
        self.target = max(0, min(100, int(target)))
        self.mode = MODE_POSITION
        self._reset_stall_timer()

    def move_direction(self, direction):
        self.direction_command = direction
        self.mode = MODE_DIRECTION
        self._reset_stall_timer()

    def stop(self):
        self.driver.stop()
        self.mode = MODE_IDLE
        self.direction_command = DIR_STOP
        self._reset_stall_timer()

    def update(self):
        # 목표 위치 제어
        if self.mode == MODE_POSITION:
            error = self.target - self.encoder.position

            if abs(error) <= POSITION_DEADBAND:
                print("[도착]", self.encoder.position, "%")
                self.stop()
                return

            if self._check_stall():
                return

            if error > 0:
                self.encoder.direction = 1
                self.driver.forward(FULL_PWM)
            else:
                self.encoder.direction = -1
                self.driver.backward(FULL_PWM)

            return

        # 리모컨 길게 누르기 방향 제어
        if self.mode == MODE_DIRECTION:
            if self.direction_command == DIR_UP:
                # 0%가 완전 올림
                if self.encoder.position <= 0:
                    self.stop()
                    return

                if self._check_stall():
                    return

                self.encoder.direction = -1
                self.driver.backward(FULL_PWM)
                return

            if self.direction_command == DIR_DOWN:
                # 100%가 완전 내림
                if self.encoder.position >= 100:
                    self.stop()
                    return

                if self._check_stall():
                    return

                self.encoder.direction = 1
                self.driver.forward(FULL_PWM)
                return

            self.stop()
            return

        self.driver.stop()


# ============================================================
# 7. 하드웨어 초기화
# ============================================================

xbee = UART(
    1,
    baudrate=XBEE_BAUD,
    bits=8,
    parity=None,
    stop=1,
    tx=Pin(XBEE_TX_GPIO),
    rx=Pin(XBEE_RX_GPIO),
    timeout=20,
)

driver = TB6612FNG(
    MOTOR_PWM_GPIO,
    MOTOR_IN1_GPIO,
    MOTOR_IN2_GPIO,
    MOTOR_STBY_GPIO,
)

encoder = EncoderAOnly(ENCODER_A_GPIO, TOTAL_PULSES)
lower_blind = LowerBlindMotor(driver, encoder)

rx_buffer = bytearray()

# 0%=최상단, 100%=최하단
# 저장된 제한값보다 더 아래로는 내려가지 않음
max_down_position = 100

last_sent_position = -1
last_status_time = ticks_ms()

last_saved_count = -1
last_save_time = ticks_ms()


# ============================================================
# 8. 상태 저장
# ============================================================

def load_state():
    global max_down_position

    try:
        with open(STATE_FILE, "r") as file:
            state = json.load(file)

        saved_position = int(state.get("position", 0))
        saved_limit = int(state.get("max_down_position", 100))

        encoder.restore_position(saved_position)
        max_down_position = max(0, min(100, saved_limit))

        print(
            "[상태 복원]",
            "위치=", encoder.position,
            "count=", encoder.count,
            "하강 제한=", max_down_position,
        )

    except OSError:
        print("[상태 복원] 저장 파일 없음 → 0% 시작")

    except Exception as error:
        print("[상태 복원 실패]", error)


def save_state(force=False):
    global last_saved_count, last_save_time

    now = ticks_ms()

    if not force:
        if encoder.count == last_saved_count:
            return

        if ticks_diff(now, last_save_time) < STATE_SAVE_INTERVAL_MS:
            return

    try:
        state = {
            "position": encoder.position,
            "max_down_position": max_down_position,
        }

        with open(STATE_FILE, "w") as file:
            json.dump(state, file)

        last_saved_count = encoder.count
        last_save_time = now

    except Exception as error:
        print("[상태 저장 실패]", error)


# ============================================================
# 9. 실제 위치를 상단 본체로 송신
# ============================================================

def send_position(force=False):
    global last_sent_position, last_status_time

    position = encoder.position
    now = ticks_ms()

    changed = position != last_sent_position
    change_interval_passed = (
        ticks_diff(now, last_status_time) >= STATUS_CHANGE_INTERVAL_MS
    )
    heartbeat_due = (
        ticks_diff(now, last_status_time) >= STATUS_HEARTBEAT_MS
    )

    if force or heartbeat_due or (changed and change_interval_passed):
        xbee.write(bytes((STATUS_LOWER_BLIND_POSITION, position)))
        last_sent_position = position
        last_status_time = now


# ============================================================
# 10. XBee 명령 처리
# ============================================================

def handle_command(cmd_id, value):
    global max_down_position

    print(
        "[XBee RX]",
        "CMD=0x{:02X}".format(cmd_id),
        "VALUE=0x{:02X}".format(value),
    )

    # 리모컨 0번 / 전체 긴급 정지
    if cmd_id == CMD_STOP:
        print("[긴급정지] 하단 블라인드 정지")
        lower_blind.stop()
        save_state(force=True)
        send_position(force=True)
        return

    # 키 3·6 짧게 또는 앱/프리셋 목표 위치
    if cmd_id == CMD_LOWER_BLIND_POSITION:
        target = max(0, min(100, int(value)))

        if target > max_down_position:
            print(
                "[하강 제한]",
                target,
                "% →",
                max_down_position,
                "%",
            )
            target = max_down_position

        print(
            "[위치 제어]",
            encoder.position,
            "% →",
            target,
            "%",
        )

        # 목표값은 현재 위치로 저장하지 않는다.
        lower_blind.move_to(target)
        send_position(force=True)
        return

    # 키 3·6 길게 누르기 및 RELEASE 정지
    if cmd_id == CMD_LOWER_BLIND_DIRECTION:
        if value == DIR_UP:
            print("[방향 제어] 하단 블라인드 올림")
            lower_blind.move_direction(DIR_UP)

        elif value == DIR_DOWN:
            print("[방향 제어] 하단 블라인드 내림")
            lower_blind.move_direction(DIR_DOWN)

        elif value == DIR_STOP:
            print("[방향 제어] 하단 블라인드 정지")
            lower_blind.stop()
            save_state(force=True)
            send_position(force=True)

        else:
            print("[무시] 알 수 없는 방향 값:", value)

        return

    # 리모컨 키 3 또는 6 더블탭
    if cmd_id == CMD_MIN_HEIGHT_SET:
        if value == LOWER_BLIND_MOTOR_ID:
            max_down_position = encoder.position
            print("[제한 저장] 하강 제한:", max_down_position, "%")
            save_state(force=True)
        return

    # 현재 리모컨 코드에서는 전송하지 않지만 앱/향후 기능용으로 지원
    if cmd_id == CMD_MIN_HEIGHT_RESET:
        if value == LOWER_BLIND_MOTOR_ID:
            max_down_position = 100
            print("[제한 초기화] 하강 제한 100%")
            save_state(force=True)
        return

    # 상단 블라인드, 하단 본체, 프리셋 등은 하단 Arduino가 처리하지 않음
    print("[무시] 하단 블라인드용 명령이 아님")


def receive_commands():
    if not xbee.any():
        return

    data = xbee.read()

    if not data:
        return

    rx_buffer.extend(data)

    # 라즈베리파이와 동일한 2바이트 고정 패킷
    while len(rx_buffer) >= 2:
        cmd_id = rx_buffer[0]
        value = rx_buffer[1]
        del rx_buffer[:2]

        handle_command(cmd_id, value)


# ============================================================
# 11. 안전 종료
# ============================================================

def safe_shutdown(reason):
    print("[안전 종료]", reason)

    try:
        lower_blind.stop()
        save_state(force=True)
        send_position(force=True)
    except Exception as error:
        print("[종료 처리 오류]", error)

    driver.disable()


# ============================================================
# 12. 메인 루프
# ============================================================

def main():
    global last_saved_count

    load_state()
    last_saved_count = encoder.count

    lower_blind.stop()
    send_position(force=True)

    print("==============================================")
    print(" 하단 본체: 하단 블라인드 전용 제어 시작")
    print(" XBee UART:", XBEE_BAUD, "bps")
    print(" 현재 위치:", encoder.position, "%")
    print(" 전체 펄스:", TOTAL_PULSES)
    print(" 처리 명령: 0x00 / 0x03 / 0x31 / 0xB0 / 0xB1")
    print("==============================================")

    previous_mode = lower_blind.mode
    last_debug_time = ticks_ms()

    try:
        while True:
            receive_commands()
            lower_blind.update()

            # 실제 엔코더 위치만 보고·저장
            send_position()
            save_state()

            # 이동 완료 또는 안전 정지 직후 즉시 상태 반영
            if (
                previous_mode != MODE_IDLE
                and lower_blind.mode == MODE_IDLE
            ):
                save_state(force=True)
                send_position(force=True)

            previous_mode = lower_blind.mode

            now = ticks_ms()
            if ticks_diff(now, last_debug_time) >= 1000:
                last_debug_time = now
                print(
                    "[DBG]",
                    "mode=", lower_blind.mode,
                    "count=", encoder.count,
                    "position=", encoder.position,
                    "target=", lower_blind.target,
                )

            sleep_ms(10)

    except KeyboardInterrupt:
        safe_shutdown("Ctrl+C")

    except Exception as error:
        safe_shutdown("실행 오류: {}".format(error))
        raise

    finally:
        driver.disable()


main()
