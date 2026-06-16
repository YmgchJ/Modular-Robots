import socket
import threading
import time
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation

SERVER_IP = "0.0.0.0"
BASE_PORT = 5000

WINDOW = 200  # 表示するサンプル数

# agent_id ごとのデータバッファ
data_lock = threading.Lock()
agent_data = {}  # {agent_id: {"angle": deque, "flex": deque, "light": deque, "current": deque, "time": deque}}

def init_agent(agent_id):
    if agent_id not in agent_data:
        agent_data[agent_id] = {
            "angle":   deque([0.0] * WINDOW, maxlen=WINDOW),
            "flex":    deque([0.0] * WINDOW, maxlen=WINDOW),
            "light":   deque([0.0] * WINDOW, maxlen=WINDOW),
            "current": deque([0.0] * WINDOW, maxlen=WINDOW),
            "time":    deque([0.0] * WINDOW, maxlen=WINDOW),
        }

def parse_status(text):
    """STATUS,id:2,angle:110.0,flex:0.1234,light:2048,current:1024"""
    d = {}
    for part in text.split(","):
        if ":" in part:
            k, v = part.split(":", 1)
            d[k.strip()] = v.strip()
    return d

def receiver():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((SERVER_IP, BASE_PORT + 200))  # モニター専用ポート
    sock.settimeout(0.5)
    print(f"[MONITOR] Listening on port {BASE_PORT + 200}")
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            text = data.decode(errors="replace").strip("\x00")
            if not text.startswith("STATUS"):
                continue
            d = parse_status(text)
            agent_id = int(d.get("id", -1))
            if agent_id < 0:
                continue
            t = time.time()
            with data_lock:
                init_agent(agent_id)
                agent_data[agent_id]["angle"].append(float(d.get("angle", 0)))
                agent_data[agent_id]["flex"].append(float(d.get("flex", 0)))
                agent_data[agent_id]["light"].append(float(d.get("light", 0)))
                agent_data[agent_id]["current"].append(float(d.get("current", 0)))
                agent_data[agent_id]["time"].append(t)
        except socket.timeout:
            pass

# --- GUI ---
fig, axes = plt.subplots(4, 1, figsize=(10, 8), sharex=False)
fig.suptitle("Pico Real-time Monitor")
labels = ["Servo Angle (deg)", "Flex Sensor", "Light Sensor (raw)", "Current (raw)"]
keys   = ["angle", "flex", "light", "current"]
colors = ["tab:blue", "tab:orange", "tab:green", "tab:red"]

lines = {}  # {agent_id: [line0, line1, line2, line3]}

def animate(_):
    with data_lock:
        aids = list(agent_data.keys())

    for agent_id in aids:
        if agent_id not in lines:
            lines[agent_id] = []
            for ax, color, label in zip(axes, colors, labels):
                line, = ax.plot([], [], color=color,
                                label=f"agent {agent_id}", linewidth=1.2)
                lines[agent_id].append(line)
                ax.set_ylabel(label, fontsize=8)
                ax.legend(loc="upper left", fontsize=7)
                ax.grid(True, alpha=0.3)

        with data_lock:
            buf = agent_data[agent_id]
            xs = list(range(WINDOW))
            values = [
                list(buf["angle"]),
                list(buf["flex"]),
                list(buf["light"]),
                list(buf["current"]),
            ]

        for i, (line, vals) in enumerate(zip(lines[agent_id], values)):
            line.set_data(xs, vals)
            axes[i].relim()
            axes[i].autoscale_view()

    return [l for ls in lines.values() for l in ls]

threading.Thread(target=receiver, daemon=True).start()

ani = animation.FuncAnimation(fig, animate, interval=100, blit=False)
plt.tight_layout()
plt.show()