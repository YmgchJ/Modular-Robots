import socket
import threading
import time 
import struct

SERVER_IP = "0.0.0.0"
BASE_PORT = 5000

# パラメータ設定（必要に応じて変更）
params = {
    "omega": 9.42,
    "kappa": 1.0,
    "center": 110.0,
    "amplitude": 60.0,
    "stop_id": 0,
    "stop_delay": 0,
    "feedback_tau": 1.0,
    "prc_n": 1,
    "prc_a0": 0.0, "prc_b0": 0.0,
    "prc_a1": 1.0, "prc_b1": 0.0,
}

def build_param_response():
    p = params
    s = (f"omega:{p['omega']},kappa:{p['kappa']},"
         f"center:{p['center']},amplitude:{p['amplitude']},"
         f"stop_id:{p['stop_id']},stop_delay:{p['stop_delay']},"
         f"feedback_tau:{p['feedback_tau']},"
         f"prc_n:{p['prc_n']},prc_a0:{p['prc_a0']},prc_b0:{p['prc_b0']},"
         f"prc_a1:{p['prc_a1']},prc_b1:{p['prc_b1']}")
    return s.encode()

# agent_id → IP を動的に学習
agent_ip_map = {}
agent_ip_lock = threading.Lock()
agent_state = {}

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((SERVER_IP, BASE_PORT))
sock.settimeout(0.5)

agent_socks = {}
agent_socks_lock = threading.Lock()

def get_or_create_agent_sock(agent_id):
    with agent_socks_lock:
        if agent_id not in agent_socks:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind((SERVER_IP, BASE_PORT + agent_id))
            s.settimeout(0.5)
            agent_socks[agent_id] = s
            threading.Thread(
                target=receiver_agent, args=(agent_id, s), daemon=True
            ).start()
            print(f"[INFO] Listening on port {BASE_PORT + agent_id} for agent {agent_id}")
        return agent_socks[agent_id]

def handle_packet(data, addr, response_sock):
    text = ""
    try:
        text = data.decode(errors='replace').strip('\x00')
    except Exception:
        pass

    # STATUSの転送
    if text.startswith("STATUS"):
        sock.sendto(data, ("127.0.0.1", MONITOR_PORT))
        return

    # HELLOハンドシェイク
    if text.startswith("HELLO:"):
        try:
            agent_id = int(text.split(":")[1])
            with agent_ip_lock:
                agent_ip_map[agent_id] = addr[0]
                agent_state.setdefault(agent_id, False)
            get_or_create_agent_sock(agent_id)
            print(f"[HANDSHAKE] agent={agent_id} IP={addr[0]} registered")
        except ValueError:
            pass
        response_sock.sendto(b"READY", addr)
        return

    # パラメータリクエスト
    if text.startswith("REQUEST_PARAMS"):
        try:
            for part in text.split(","):
                if part.startswith("id:"):
                    agent_id = int(part.split(":")[1])
                    with agent_ip_lock:
                        agent_ip_map[agent_id] = addr[0]
                        agent_state.setdefault(agent_id, False)
                    get_or_create_agent_sock(agent_id)
                    break
        except:
            pass
        resp = build_param_response()
        response_sock.sendto(resp, addr)
        print(f"[PARAMS] Sent params to {addr}: {resp.decode()[:60]}...")
        return

    # ログデータ（バイナリ）
    if len(data) >= 5 and data[0] < 128:
        agent_id = data[0]
        num_records = (len(data) - 5) // 6
        print(f"[LOG] agent={agent_id} records={num_records} from {addr}")

        with agent_ip_lock:
            agent_ip_map[agent_id] = addr[0]

        if num_records > 0:
            last_offset = 5 + (num_records - 1) * 6
            micros24_bytes = data[last_offset:last_offset + 3]
            micros24 = micros24_bytes[0] | (micros24_bytes[1] << 8) | (micros24_bytes[2] << 16)
            ack = bytes([agent_id,
                         micros24 & 0xFF,
                         (micros24 >> 8) & 0xFF,
                         (micros24 >> 16) & 0xFF])
            # agent専用ソケットで返す
            with agent_socks_lock:
                s = agent_socks.get(agent_id)
            if s:
                s.sendto(ack, addr)
                print(f"[ACK] → {addr} via agent sock")
            else:
                response_sock.sendto(ack, addr)
                print(f"[ACK] → {addr} via response sock")
        return

    print(f"[RX] {addr}: {text[:60]}")

def receiver_main():
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            handle_packet(data, addr, sock)
        except socket.timeout:
            pass

def receiver_agent(agent_id, s):
    while True:
        try:
            data, addr = s.recvfrom(1024)
            handle_packet(data, addr, s)
        except socket.timeout:
            pass

def staircase_on():
    """5秒後から1秒間隔でagent番号順にON、最後のagentがONになって10秒後に全台OFF"""
    with agent_ip_lock:
        aids = sorted(agent_ip_map.keys())
    
    if not aids:
        print("[ERROR] No agents discovered yet")
        return
    
    print(f"[STAIRCASE] Starting in 5 seconds... order: {aids}")
    
    def _run():
        time.sleep(5)
        for i, aid in enumerate(aids):
            send_command(aid, "ON")
            with agent_ip_lock:
                agent_state[aid] = True
            print(f"[STAIRCASE] agent {aid} ON ({i+1}/{len(aids)})")
            if i < len(aids) - 1:
                time.sleep(1)
        
        print(f"[STAIRCASE] All agents ON. Stopping all in 10 seconds...")
        time.sleep(10)
        
        send_command_all("OFF")
        with agent_ip_lock:
            for aid in agent_ip_map:
                agent_state[aid] = False
        print("[STAIRCASE] All agents OFF")
    
    threading.Thread(target=_run, daemon=True).start()

def send_command(agent_id, cmd):
    with agent_ip_lock:
        ip = agent_ip_map.get(agent_id)
    if ip is None:
        print(f"[ERROR] agent {agent_id} not yet discovered")
        return
    agent_port = BASE_PORT + agent_id
    msg = f"{agent_id}:{cmd}".encode()
    with agent_socks_lock:
        s = agent_socks.get(agent_id, sock)
    s.sendto(msg, (ip, agent_port))
    print(f"[TX] → {ip}:{agent_port}  {msg}")

def set_param(key, value):
    if key in params:
        try:
            params[key] = float(value)
            print(f"[PARAM] {key} = {params[key]}")
        except:
            print("invalid value")
    else:
        print(f"unknown param: {key}. available: {list(params.keys())}")

def send_command_all(cmd):
    """登録済み全agentに送信"""
    with agent_ip_lock:
        aids = list(agent_ip_map.keys())
    if not aids:
        print("[ERROR] No agents discovered yet")
        return
    for aid in aids:
        send_command(aid, cmd)

def control_loop():
    print("Commands:")
    print("  <id> ON/OFF        : 個別制御  例) 2 ON")
    print("  2 4 6 ON/OFF       : 複数同時  例) 2 4 ON")
    print("  all ON/OFF         : 全台同時")
    print("  staircase          : 5秒後から1秒間隔で順番にON")
    print("  list               : 登録済みagent一覧")
    print("  set <key> <value>  : パラメータ変更")
    print("  params             : 現在のパラメータ表示")

    while True:
        cmd = input("cmd: ").strip()
        if not cmd:
            continue

        if cmd == "list":
            with agent_ip_lock:
                if not agent_ip_map:
                    print("  (no agents)")
                for aid, ip in agent_ip_map.items():
                    print(f"  agent {aid}: {ip}  state={agent_state.get(aid)}")
            continue

        if cmd == "params":
            print(params)
            continue

        # staircase
        if cmd == "staircase":
            staircase_on()
            continue

        parts = cmd.split()

        # set <key> <value>
        if len(parts) == 3 and parts[0] == "set":
            set_param(parts[1], parts[2])
            continue

        # all ON/OFF
        if len(parts) == 2 and parts[0] == "all":
            state = parts[1].upper()
            if state in ["ON", "OFF"]:
                send_command_all(state)
                with agent_ip_lock:
                    for aid in agent_ip_map:
                        agent_state[aid] = (state == "ON")
            else:
                print("use ON/OFF")
            continue

        # <id> [id2 id3 ...] ON/OFF
        if len(parts) >= 2 and parts[-1].upper() in ["ON", "OFF"]:
            state = parts[-1].upper()
            id_strs = parts[:-1]
            valid = True
            aids = []
            for s in id_strs:
                try:
                    aids.append(int(s))
                except ValueError:
                    print(f"invalid id: {s}")
                    valid = False
                    break
            if valid:
                for aid in aids:
                    send_command(aid, state)
                with agent_ip_lock:
                    for aid in aids:
                        agent_state[aid] = (state == "ON")
            continue

        print("format: <id> [id2 ...] ON/OFF | all ON/OFF | staircase | list | set <key> <value> | params")

threading.Thread(target=receiver_main, daemon=True).start()
print(f"[INFO] Server started on port {BASE_PORT}")
control_loop()