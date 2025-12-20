# packet_utils.py

SUPPORTED_COMMANDS = {
    "ENC": 2,   # Encoder data: enc_L, enc_R
    "ESD": 4,   # Ultrasonic sensor data: es1 ~ es6
    #"YOL": 2,   # Yolo data: distance, angular
    # 나중에 여기에 "IMU": 3, "GPS": 4 등 추가 가능
}

def parse_packet(packet: str):
    packet = packet.strip()

    # 시작과 끝 검사
    if not (packet.startswith('!') and packet.endswith('?')):
        return None

    try:
        # !COMMAND/COUNT/DATA1/.../DATAn?
        content = packet[1:-1]  # '!'와 '?' 제거
        parts = content.split('/')

        command = parts[0]
        field_count = int(parts[1])
        data_fields = parts[2:]

        # 지원하지 않는 명령어이거나 필드 개수 불일치 시 무시
        if command not in SUPPORTED_COMMANDS:
            return None
        if field_count != SUPPORTED_COMMANDS[command]:
            return None
        if len(data_fields) != field_count:
            return None

        return {
            'command': command,
            'fields': data_fields
        }

    except Exception:
        return None
