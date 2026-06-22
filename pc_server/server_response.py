# サーバー側で管理するパラメータ
# エージェントIDごとのomega値を配列で管理
import math
import os
import re

omega_values = {
    4: 3.14 * 2.7    # エージェント1の周波数
}
default_omega = 3.14 * 2.50 # デフォルト周波数（未定義IDの場合）

kappa = 3       # フィードバックゲイン
alpha = -3.14*0.6
servo_center = 90.0  # サーボ中心角度
servo_amplitude = 65.0 # サーボ振幅
stop_agent_id = 4      # 停止対象のエージェントID (0の場合はどのも停止しない等を意味づけることも可能)
stop_delay_seconds = 30000 # 停止までの秒数
feedback_tau_sec = 1.0  # 一次遅れフィルタの時定数 [s]

# PRCのフーリエ係数（0..prc_harmonics を使用）
# z(psi) = Σ [ prc_a[n] * cos(n*psi) + prc_b[n] * sin(n*psi) ]
PRC_SOURCE_DIR = os.environ.get(
    "PRC_SOURCE_DIR",
    os.path.join("EstimateQ", "Spring3", "240_0531", "gamma_exports")
)
PRC_SOURCE_FILE = "prc_snippet_ref_w2.txt"


def load_prc_from_directory(source_dir, source_file=PRC_SOURCE_FILE):
    """PRC snippet txt からフーリエ係数を読み込む。"""
    source_path = os.path.join(source_dir, source_file)
    if not os.path.isfile(source_path):
        raise FileNotFoundError(f"PRC snippet file not found: {source_path}")

    with open(source_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    harmonics = None
    harmonics_pattern = re.compile(r"^\s*prc_harmonics\s*=\s*(\d+)\s*$")
    for line in lines:
        m = harmonics_pattern.match(line)
        if m:
            harmonics = int(m.group(1))
            break

    if harmonics is None:
        raise ValueError(f"prc_harmonics not found in {source_path}")

    a = [0.0] * (harmonics + 1)
    b = [0.0] * (harmonics + 1)
    num_pattern = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
    a_pattern = re.compile(rf"^\s*prc_a\[(\d+)\]\s*=\s*({num_pattern})\s*$")
    b_pattern = re.compile(rf"^\s*prc_b\[(\d+)\]\s*=\s*({num_pattern})\s*$")

    for line in lines:
        m_a = a_pattern.match(line)
        if m_a:
            idx = int(m_a.group(1))
            if 0 <= idx <= harmonics:
                a[idx] = float(m_a.group(2))
            continue

        m_b = b_pattern.match(line)
        if m_b:
            idx = int(m_b.group(1))
            if 0 <= idx <= harmonics:
                b[idx] = float(m_b.group(2))

    return harmonics, a, b


try:
    prc_harmonics, prc_a, prc_b = load_prc_from_directory(PRC_SOURCE_DIR)
    print(f"[INFO] Loaded PRC from {os.path.join(PRC_SOURCE_DIR, PRC_SOURCE_FILE)}")
except Exception as e:
    raise RuntimeError(
        f"Failed to load PRC from directory '{PRC_SOURCE_DIR}' "
        f"(file: {PRC_SOURCE_FILE}). {e}"
    ) from e




def build_prc_payload():
    """PRC係数をレスポンス文字列へ展開"""
    fields = [f"prc_n:{prc_harmonics}"]
    for n in range(0, prc_harmonics + 1):
        fields.append(f"prc_a{n}:{prc_a[n]:.6f}")
        fields.append(f"prc_b{n}:{prc_b[n]:.6f}")
    return ",".join(fields)


def format_payload_for_log(payload, max_prc_order=3):
    """payload文字列を横長で見やすい形式に整形（PRCはmax_prc_order次まで表示）"""
    fields = []
    for item in payload.split(','):
        if ':' in item:
            key, value = item.split(':', 1)
            fields.append((key, value))

    base_items = []
    prc_n = None
    prc_items = {}

    for key, value in fields:
        if key == "prc_n":
            prc_n = value
        elif key.startswith("prc_a") or key.startswith("prc_b"):
            suffix = key[5:]
            if suffix.isdigit():
                idx = int(suffix)
                if idx <= max_prc_order:
                    prc_items[key] = value
        else:
            base_items.append(f"{key}={value}")

    prc_parts = []
    if prc_n is not None:
        prc_parts.append(f"prc_n={prc_n}")
    for n in range(0, max_prc_order + 1):
        a_key = f"prc_a{n}"
        b_key = f"prc_b{n}"
        if a_key in prc_items:
            prc_parts.append(f"a{n}={prc_items[a_key]}")
        if b_key in prc_items:
            prc_parts.append(f"b{n}={prc_items[b_key]}")

    base_str = ", ".join(base_items)
    prc_str = ", ".join(prc_parts)
    if prc_str:
        prc_str += ", ..."
    return f"{base_str} | {prc_str}" if prc_str else base_str

def get_omega_for_agent(agent_id):
    """
    エージェントIDに応じたomega値を取得する関数
    """
    return omega_values.get(agent_id, default_omega)

def handle_handshake(sock, data, addr):
    """
    クライアントからのハンドシェイクメッセージに応答する関数。
    """
    handshake_message = "HELLO"
    try:
        if data.decode('utf-8') == handshake_message:
            response = "READY"
            sock.sendto(response.encode('utf-8'), addr)
            print(f"[INFO] Handshake response sent to {addr}")
    except UnicodeDecodeError:
        print(f"[WARN] Received non-UTF-8 data from {addr}, ignoring.")

def handle_parameter_request(sock, data, addr):
    """
    パラメータリクエストを処理し、デバッグ情報を表示
    """
    request_str = data.decode('utf-8')

    # リクエストデータを解析
    if request_str.startswith("REQUEST_PARAMS"):
        try:
            # デバッグ情報を解析
            parts = request_str.split(',')
            agent_id = int(parts[1].split(':')[1])  # id:<value>
            analog26 = int(parts[2].split(':')[1])  # analog26:<value>

            # analog26 を電圧値に変換
            voltage = (analog26 / 4095) * 3.3 * 2

            # エージェントIDに応じたomega値を取得
            omega = get_omega_for_agent(agent_id)

            # サーバー側のパラメータを送信
            response = (
                f"omega:{omega:.2f},kappa:{kappa:.2f},"
                f"center:{servo_center:.1f},amplitude:{servo_amplitude:.1f},"
                f"stop_id:{stop_agent_id},stop_delay:{stop_delay_seconds},"
                f"feedback_tau:{feedback_tau_sec:.3f},"
                f"{build_prc_payload()}"
            )
            sock.sendto(response.encode('utf-8'), addr)
            print(
                f"[INFO] Sent parameters | id={agent_id}, O={omega:.2f}, V={voltage:.2f} | "
                f"{format_payload_for_log(response, max_prc_order=2)}"
            )
            return agent_id

        except (IndexError, ValueError) as e:
            print(f"[ERROR] Failed to parse parameter request: {request_str}")
            print(f"[ERROR] {e}")
    else:
        print(f"[WARN] Invalid parameter request from {addr}: {request_str}")