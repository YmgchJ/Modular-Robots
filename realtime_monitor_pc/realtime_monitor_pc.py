"""
request_data.py

物理ボタンのみで動くPico（realtime_monitor.ino）に対して、
「直近のトライアルのデータが欲しい」という要求を送り、
受信したデータをCSVに保存するシンプルなスクリプトです。

Picoは記録中(motionMode中)はWiFiを完全に切断しているため、
PCから一方的に「いつ繋がっているか」を知ることはできません。
そのため、このスクリプトは下記の PICO_IP を手動で指定する必要があります
（Picoがボタンで停止し、WiFiに再接続した直後にシリアルモニタへ表示される
 IPアドレスを確認して、ここに設定してください）。

使い方:
  1. 下記の PICO_IP を、Picoが停止後に再接続したIPアドレスに変更する
  2. python request_data.py を実行する
  3. "REQUEST_DATA" を送信し、応答(DUMPSTART〜DUMPEND)を待つ
  4. 受信が完了すると、trials/ フォルダにCSVが保存される
"""

import csv
import os
import socket
import time
from datetime import datetime

PICO_IP = "192.168.0.125"   # ← Picoが再接続後に表示するIPアドレスに変更してください
COMMAND_PORT = 6001        # Pico側の commandPort と合わせる
LISTEN_PORT = 6000         # Pico側の pcPort と合わせる（データ受信用）

CSV_OUTPUT_DIR = "trials"
DUMP_WAIT_TIMEOUT_SEC = 60.0  # REQUEST_DATA送信後、転送が始まるのを待つ最大時間


def send_request(sock):
    sock.sendto(b"REQUEST_DATA", (PICO_IP, COMMAND_PORT))
    print(f"[CMD] Sent 'REQUEST_DATA' to {PICO_IP}:{COMMAND_PORT}")


def save_csv(t_list, angle_list, flex_list, cur_list):
    if len(t_list) == 0:
        print("[CSV] No data received. Skipping save.")
        return

    os.makedirs(CSV_OUTPUT_DIR, exist_ok=True)
    filename = datetime.now().strftime("trial_%Y%m%d_%H%M%S.csv")
    filepath = os.path.join(CSV_OUTPUT_DIR, filename)

    with open(filepath, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["elapsed_sec", "angle_deg", "flex_raw_adc", "current_mA"])
        for row in zip(t_list, angle_list, flex_list, cur_list):
            writer.writerow(row)

    print(f"[CSV] Saved {len(t_list)} rows to {filepath}")


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", LISTEN_PORT))
    sock.settimeout(1.0)

    send_request(sock)

    dump_active = False
    t_list, angle_list, flex_list, cur_list = [], [], [], []

    start_wait = time.time()
    print(f"[INFO] Waiting for response (timeout {DUMP_WAIT_TIMEOUT_SEC:.0f}s)...")

    while True:
        if time.time() - start_wait > DUMP_WAIT_TIMEOUT_SEC and not dump_active:
            print("[ERROR] No response from Pico. Check PICO_IP and that the trial has "
                  "finished (button pressed to stop) so WiFi is reconnected.")
            return

        try:
            data, addr = sock.recvfrom(4096)
        except socket.timeout:
            continue

        text = data.decode("utf-8", errors="ignore").strip()

        if text == "DUMPSTART":
            dump_active = True
            t_list, angle_list, flex_list, cur_list = [], [], [], []
            print("[DUMP] Transfer started. Receiving data...")
            continue

        if text == "DUMPEND":
            print(f"[DUMP] Transfer finished. Received {len(t_list)} samples.")
            save_csv(t_list, angle_list, flex_list, cur_list)
            return

        if text.startswith("BATCH:"):
            lines = text.splitlines()
            header = lines[0]
            try:
                batch_id = header.split(":", 1)[1]
            except IndexError:
                continue

            count = 0
            for data_line in lines[1:]:
                parts = data_line.split(",")
                if len(parts) != 4:
                    continue
                try:
                    t_ms = float(parts[0])
                    angle_v = float(parts[1])
                    flex_v = float(parts[2])
                    current_centi = float(parts[3])
                except ValueError:
                    continue
                t_list.append(t_ms / 1000.0)
                angle_list.append(angle_v)
                flex_list.append(flex_v)
                cur_list.append(current_centi / 10.0)
                count += 1

            ack_msg = f"ACK:{batch_id}"
            sock.sendto(ack_msg.encode("utf-8"), (addr[0], COMMAND_PORT))
            print(f"[DUMP] Batch {batch_id}: {count} samples received, ACK sent. "
                  f"(total so far: {len(t_list)})")


if __name__ == "__main__":
    main()