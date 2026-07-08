# receiver_json.py
# Ubuntu PC에서 실행: Windows 노트북이 보내는 JSON 탐지 결과를 받는 수신기
# 실행: python3 receiver_json.py

import socket
import json

HOST = "0.0.0.0"   # 0.0.0.0 = "내 컴퓨터의 모든 네트워크 카드에서 수신"
PORT = 5000        # 사용할 포트 번호 (송신 쪽과 반드시 같아야 함)


def main():
    # TCP 서버 소켓 생성 (AF_INET=IPv4, SOCK_STREAM=TCP)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # 프로그램을 껐다 켰을 때 "포트가 이미 사용 중" 오류 방지
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))   # 5000번 포트에 자리 잡기
    server.listen(1)            # 연결 대기 시작 (동시 접속 1개)
    print(f"[RECEIVER] Listening on {HOST}:{PORT}")

    while True:
        # 송신 쪽이 접속해 올 때까지 여기서 멈춰서 기다림
        conn, addr = server.accept()
        print(f"[RECEIVER] Connected from {addr}")

        buffer = ""  # TCP는 데이터가 잘려서 올 수 있어 임시 저장소 필요
        with conn:
            while True:
                data = conn.recv(1024)      # 최대 1024바이트씩 수신
                if not data:                # 빈 데이터 = 상대가 연결 끊음
                    print("[RECEIVER] Disconnected. Waiting for new connection...")
                    break

                buffer += data.decode("utf-8")

                # 메시지 구분자인 줄바꿈(\n)이 나올 때마다 한 건씩 처리
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    if not line.strip():
                        continue
                    try:
                        msg = json.loads(line)   # JSON 문자열 -> 파이썬 딕셔너리
                        print(f"[RECEIVER] Received: {msg}")
                        # TODO(Phase 3): 여기서 msg의 좌표를
                        # ROS 2 /goal_pose 발행으로 연결하면 됨
                    except json.JSONDecodeError:
                        print(f"[RECEIVER] Bad JSON: {line}")


if __name__ == "__main__":
    main()
