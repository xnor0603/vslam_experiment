#!/usr/bin/env python3
"""三張軌跡圖：vehicle_local_position / vehicle_gps_position / VSLAM odometry。

底層邏輯：三個來源的座標系不同，不能直接疊，各自獨立做俯視軌跡圖。
時間軸著色用 viridis 對齊既有風格。VSLAM panel 已 invert x 軸對齊物理視角。

用法:
    python3 plot_three_trajectories.py <source_bag_dir_or_db3> <vslam_bag_dir_or_db3>
    # 不傳參數時會用預設（最近一次分析的 bag 對）
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

DEFAULT_TRIM = '/home/landis/VSLAM/isaac_ros-dev/bags/rosbag2_2026_04_29-10_24_28_trimmed/rosbag2_2026_04_29-10_24_28_trimmed_0.db3'
DEFAULT_VSLAM = '/home/landis/VSLAM/isaac_ros-dev/bags/vslam_output_20260429_151725/vslam_output_20260429_151725_0.db3'

def resolve_bag_db(path):
    """接受 bag 目錄或 .db3 檔；目錄則自動找裡面的 .db3。"""
    if os.path.isfile(path) and path.endswith('.db3'):
        return path
    if os.path.isdir(path):
        cand = glob.glob(os.path.join(path, '*.db3'))
        if cand: return cand[0]
    raise FileNotFoundError(f'找不到 .db3：{path}')

DB_TRIM = resolve_bag_db(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_TRIM
DB_VSLAM = resolve_bag_db(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_VSLAM
TRIM_NAME = os.path.basename(os.path.dirname(DB_TRIM))
VSLAM_NAME = os.path.basename(os.path.dirname(DB_VSLAM))
print(f'source bag : {TRIM_NAME}')
print(f'vslam bag  : {VSLAM_NAME}')

def bag_start_timestamp(db_path):
    """讀 bag 第一個訊息的 timestamp（ns）格式化成 YYYYMMDD_HHMMSS — 即資料採集時間。"""
    con = sqlite3.connect(db_path); cur = con.cursor()
    t0_ns = cur.execute('SELECT MIN(timestamp) FROM messages').fetchone()[0]
    con.close()
    return datetime.datetime.fromtimestamp(t0_ns / 1e9).strftime('%Y%m%d_%H%M%S')

TS = bag_start_timestamp(DB_TRIM)
OUT = f'/home/landis/VSLAM/ClaudeCode/figures/{TS}_trajectories.png'

def read_topic(db, topic, msg_type):
    con = sqlite3.connect(db); cur = con.cursor()
    tid = cur.execute("SELECT id FROM topics WHERE name=?", (topic,)).fetchone()[0]
    rows = cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
                       (tid,)).fetchall()
    con.close()
    return [(t, deserialize_message(bytes(b), msg_type)) for t, b in rows]

# ---- 1. vehicle_local_position (PX4 NED) ----
lp = read_topic(DB_TRIM, '/px4_2/fmu/out/vehicle_local_position', VehicleLocalPosition)
lp_t0 = lp[0][0]
lp_ts = np.array([(t - lp_t0)/NS for t, _ in lp])
lp_x = np.array([m.x for _, m in lp])  # north
lp_y = np.array([m.y for _, m in lp])  # east
lp_z = np.array([m.z for _, m in lp])  # down

# ---- 2. vehicle_gps_position (lat/lon → local NE) ----
gps = read_topic(DB_TRIM, '/px4_2/fmu/out/vehicle_gps_position', SensorGps)
gps_t0 = gps[0][0]
gps_ts = np.array([(t - gps_t0)/NS for t, _ in gps])
lat0 = gps[0][1].lat * 1e-7
lon0 = gps[0][1].lon * 1e-7
gps_n, gps_e = [], []
for _, m in gps:
    lat = m.lat * 1e-7; lon = m.lon * 1e-7
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    gps_n.append(dlat * R_EARTH)
    gps_e.append(dlon * R_EARTH * math.cos(math.radians(lat0)))
gps_n = np.array(gps_n); gps_e = np.array(gps_e)

# ---- 3. VSLAM odometry (ROS ENU, x=forward y=left z=up) ----
od = read_topic(DB_VSLAM, '/visual_slam/tracking/odometry', Odometry)
od_t0 = od[0][0]
od_ts = np.array([(t - od_t0)/NS for t, _ in od])
od_x = np.array([m.pose.pose.position.x for _, m in od])
od_y = np.array([m.pose.pose.position.y for _, m in od])

# ---- 繪圖：1x3 橫排 ----
fig, axes = plt.subplots(1, 3, figsize=(21, 7))

# panel 1: vehicle_local_position (East-North top-down, NED)
ax = axes[0]
ax.plot(lp_y, lp_x, '-', color='steelblue', linewidth=0.6, alpha=0.5, zorder=1)
sc1 = ax.scatter(lp_y, lp_x, c=lp_ts, cmap='viridis', s=8, alpha=0.85, zorder=2)
ax.set_xlabel('East / y (m)', fontsize=12)
ax.set_ylabel('North / x (m)', fontsize=12)
ax.set_title(f'/px4_2/fmu/out/vehicle_local_position\n{len(lp)} poses', fontsize=11)
ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
ax.axhline(0, color='gray', linewidth=0.5, alpha=0.5)
ax.axvline(0, color='gray', linewidth=0.5, alpha=0.5)
plt.colorbar(sc1, ax=ax, label='time (s)')

# panel 2: vehicle_gps_position (East-North local, from lat/lon)
ax = axes[1]
ax.plot(gps_e, gps_n, '-', color='steelblue', linewidth=0.6, alpha=0.5, zorder=1)
sc2 = ax.scatter(gps_e, gps_n, c=gps_ts, cmap='viridis', s=8, alpha=0.85, zorder=2)
ax.set_xlabel('East (m)', fontsize=12)
ax.set_ylabel('North (m)', fontsize=12)
ax.set_title(f'/px4_2/fmu/out/vehicle_gps_position\n{len(gps)} fixes', fontsize=11)
ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
ax.axhline(0, color='gray', linewidth=0.5, alpha=0.5)
ax.axvline(0, color='gray', linewidth=0.5, alpha=0.5)
plt.colorbar(sc2, ax=ax, label='time (s)')

# panel 3: VSLAM odometry (ENU top-down, left 朝圖上左、forward 朝圖上上 — 物理視角對齊)
ax = axes[2]
ax.plot(od_y, od_x, '-', color='steelblue', linewidth=0.6, alpha=0.5, zorder=1)
sc3 = ax.scatter(od_y, od_x, c=od_ts, cmap='viridis', s=8, alpha=0.85, zorder=2)
ax.invert_xaxis()  # 讓「left 朝圖上左」，符合物理視角而非程式慣例
ax.set_xlabel('← left   y (m)   right →', fontsize=12)
ax.set_ylabel('↑ forward   x (m)   back ↓', fontsize=12)
ax.set_title(f'/visual_slam/tracking/odometry\n{len(od)} poses', fontsize=11)
ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
ax.axhline(0, color='gray', linewidth=0.5, alpha=0.5)
ax.axvline(0, color='gray', linewidth=0.5, alpha=0.5)
plt.colorbar(sc3, ax=ax, label='time (s)')

plt.suptitle(f'Three-source trajectory comparison\nsource: {TRIM_NAME}    vslam: {VSLAM_NAME}',
             fontsize=12, fontweight='bold')
plt.tight_layout()
os.makedirs(os.path.dirname(OUT), exist_ok=True)
plt.savefig(OUT, dpi=110, bbox_inches='tight')
plt.close()
print(f'圖已存 → {OUT}')
print(f'  vehicle_local_position: {len(lp)} 筆, t={lp_ts[-1]:.1f}s')
print(f'  vehicle_gps_position : {len(gps)} 筆, t={gps_ts[-1]:.1f}s')
print(f'  VSLAM odometry       : {len(od)} 筆, t={od_ts[-1]:.1f}s')
