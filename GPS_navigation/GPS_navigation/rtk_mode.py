import serial

def enable_rtk(serial_port, baud_rate=9600):
    # 시리얼 포트 연결
    with serial.Serial(serial_port, baud_rate, timeout=1) as ser:
        # UBX-CFG-TMODE3 메시지를 통해 RTK 모드 활성화
        # 예시 메시지; 실제 메시지는 모듈 사양에 따라 다를 수 있음
        set_rtk_mode = bytes.fromhex('B5 62 06 71 28 00 00 00 01 00 00 00 00 00 00 00 01 00 00 00 FF FF FF FF 00 00 00 00 00 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 10 27 00 00 10 27 00 00 10 27 00 00 00 00 00 00 03 2A 59 B0')
        # RTK 모드 설정 명령 전송
        ser.write(set_rtk_mode)
        # 설정이 제대로 적용되었는지 확인
        response = ser.read(100)
        print('Response from GPS:', response)

if __name__ == '__main__':
    enable_rtk('/dev/ttyACM0')  # 실제 연결된 시리얼 포트로 변경
