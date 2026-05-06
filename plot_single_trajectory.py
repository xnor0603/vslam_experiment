#!/usr/bin/env python3
"""單 source 軌跡圖 — 自動偵測 bag 內第一個位置 topic（GPS / vehicle_local_position / VSLAM odometry）。

底層邏輯：sister to plot_three_trajectories.py。當 bag 只錄一個位置 topic（例如只錄 GPS），
直接複用對應 panel 的繪圖風格與座標慣例。檔名 timestamp 用 bag 第一個訊息時間（資料採集時間）。

用法:
    python3 plot_single_trajectory.py <bag_dir_or_db3>
"""
import sqlite3, math, os, sys, glob, datetime
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition, SensorGps
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

NS = 1_000_000_000
R_EARTH = 6378137.0

def resolve_bag_db(path):
    if os.path.isfile(path) and path.endswith('.db3'): return path
    if os.path.isdir(path):
        cand = glob.glob(os.path.join(path, '*.db3'))
        if cand: return cand[0]
    raise FileNotFoundError(f'找不到 .db3：{path}')

if len(sys.argv) < 2:
    print('用法: plot_single_trajectory.py <bag_dir_or_db3>'); sys.exit(1)

DB = resolve_bag_db(sys.argv[1])
BAG_NAME = os.path.basename(os.path.dirname(DB))
print(f'bag: {BAG_NAME}')

con = sqlite3.connect(DB); cur = con.cursor()
topics = cur.execute("SELECT id, name, type FROM topics").fetchall()

# 自動偵測位置相關 topic（按優先序）
TOPIC_HANDLERS = {
    'px4_msgs/msg/SensorGps': 'gps',
    'px4_msgs/msg/VehicleGpsPosition': 'gps',
    'nav_msgs/msg/Odometry': 'odom',
    'px4_msgs/msg/VehicleLocalPosition': 'local',
}
target = None
for tid, name, mtype in topics:
    if mtype in TOPIC_HANDLERS:
        target = (tid, name, mtype, TOPIC_HANDLERS[mtype]); break

if target is None:
    print(f'❌ bag 內沒有支援的位置 topic'); sys.exit(2)

tid, tname, mtype, kind = target
print(f'偵測 topic: {tname}  type={mtype}  → {kind}')

# bag 開始時間（資料採集 timestamp）
t0_ns = cur.execute('SELECT MIN(timestamp) FROM messages').fetchone()[0]
TS = datetime.datetime.fromtimestamp(t0_ns / 1e9).strftime('%Y%m%d_%H%M%S')
OUT = f'/home/landis/VSLAM/ClaudeCode/figures/{TS}_trajectory_single.png'

# 讀資料
rows = cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)).fetchall()
con.close()

# 解析 + 畫圖
fig, ax = plt.subplots(figsize=(9, 8))

if kind == 'gps':
    msgs = [deserialize_message(bytes(b), SensorGps) for _,b in rows]
    ts = np.array([(t-t0_ns)/NS for t,_ in rows])
    lat0 = msgs[0].lat * 1e-7; lon0 = msgs[0].lon * 1e-7
    n_arr, e_arr = [], []
    for m in msgs:
        dlat = math.radians(m.lat*1e-7 - lat0)
        dlon = math.radians(m.lon*1e-7 - lon0)
        n_arr.append(dlat * R_EARTH)
        e_arr.append(dlon * R_EARTH * math.cos(math.radians(lat0)))
    n_arr = np.array(n_arr); e_arr = np.array(e_arr)
    ax.plot(e_arr, n_arr, '-', color='steelblue', linewidth=0.6, alpha=0.5, zorder=1)
    sc = ax.scatter(e_arr, n_arr, c=ts, cmap='viridis', s=8, alpha=0.85, zorder=2)
    ax.set_xlabel('East (m)', fontsize=12)
    ax.set_ylabel('North (m)', fontsize=12)
    ax.set_title(f'{tname}\n{len(msgs)} fixes', fontsize=11)
elif kind == 'local':
    msgs = [deserialize_message(bytes(b), VehicleLocalPosition) for _,b in rows]
    ts = np.array([(t-t0_ns)/NS for t,_ in rows])
    xs = np.array([m.x for m in msgs])  # north
    ys = np.array([m.y for m in msgs])  # east
    ax.plot(ys, xs, '-', color='steelblue', linewidth=0.6, alpha=0.5, zorder=1)
    sc = ax.scatter(ys, xs, c=ts, cmap='viridis', s=8, alpha=0.85, zorder=2)
    ax.set_xlabel('East / y (m)', fontsize=12)
    ax.set_ylabel('North / x (m)', fontsize=12)
    ax.set_title(f'{tname}\n{len(msgs)} poses', fontsize=11)
elif kind == 'odom':
    msgs = [deserialize_message(bytes(b), Odometry) for _,b in rows]
    ts = np.array([(t-t0_ns)/NS for t,_ in rows])
    xs = np.array([m.pose.pose.position.x for m in msgs])
    ys = np.array([m.pose.pose.position.y for m in msgs])
    ax.plot(ys, xs, '-', color='steelblue', linewidth=0.6, alpha=0.5, zorder=1)
    sc = ax.scatter(ys, xs, c=ts, cmap='viridis', s=8, alpha=0.85, zorder=2)
    ax.invert_xaxis()  # 對齊物理視角
    ax.set_xlabel('← left   y (m)   right →', fontsize=12)
    ax.set_ylabel('↑ forward   x (m)   back ↓', fontsize=12)
    ax.set_title(f'{tname}\n{len(msgs)} poses', fontsize=11)

ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
ax.axhline(0, color='gray', linewidth=0.5, alpha=0.5)
ax.axvline(0, color='gray', linewidth=0.5, alpha=0.5)
plt.colorbar(sc, ax=ax, label='time (s)')
plt.suptitle(f'Single-source trajectory   bag: {BAG_NAME}', fontsize=12, fontweight='bold')
plt.tight_layout()
os.makedirs(os.path.dirname(OUT), exist_ok=True)
plt.savefig(OUT, dpi=110, bbox_inches='tight')
plt.close()
print(f'圖已存 → {OUT}')
print(f'  duration: {ts[-1]:.1f}s')
