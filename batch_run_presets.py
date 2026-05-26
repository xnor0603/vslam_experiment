#!/usr/bin/env python3
"""自動化跑多個 GPS preset 並紀錄 EKF vs Ground Truth 軌跡誤差。

流程：
  for each preset:
    1. 透過 MAVLink 套用 GPS noise preset
    2. 啟動 forward_5m_experiment.py 子行程
    3. 同步採樣 Gazebo ground truth (/world/default/dynamic_pose/info)
    4. 從 exp log 抓終點報告數字
    5. 結果寫入 CSV

執行：
  python3 batch_run_presets.py [--runs N] [preset1 preset2 ...]
不指定 preset 就跑全部；不指定 --runs 預設 1 次。
"""
import csv
import os
import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from gps_presets import PRESETS, apply_preset


REPORT_RE = re.compile(
    r"指令前進距離\s*:\s*([\d.\-]+).*?"
    r"EKF 沿 heading\s*:\s*([\d.\-]+).*?"
    r"EKF 側向漂移\s*:\s*([\d.\-+]+).*?"
    r"EKF 高度誤差\s*:\s*([\d.\-+]+)",
    re.S)


def get_gz_pose(model='x500_0'):
    """Return (x, y, z, roll, pitch, yaw) from Gazebo, or None."""
    try:
        out = subprocess.run(['gz', 'model', '--model', model, '--pose'],
                             capture_output=True, text=True, timeout=2).stdout
        lines = [l.strip() for l in out.strip().splitlines() if l.strip().startswith('[')]
        if len(lines) >= 2:
            xyz = [float(x) for x in lines[0].strip('[]').split()]
            rpy = [float(x) for x in lines[1].strip('[]').split()]
            return xyz + rpy
    except Exception:
        pass
    return None


def force_disarm():
    try:
        subprocess.run(['python3', '-c', '''
from pymavlink import mavutil
m = mavutil.mavlink_connection("udpin:0.0.0.0:14540")
m.wait_heartbeat(timeout=3)
m.mav.command_long_send(1, 1, 400, 0, 0, 21196, 0, 0, 0, 0, 0)
'''], timeout=8, capture_output=True)
    except Exception:
        pass


def run_one_preset(preset_name, runs=1, takeoff_alt=2.0, forward_dist=5.0):
    """跑一個 preset N 次，回傳 list of result dict."""
    print(f"\n{'='*60}\n== PRESET: {preset_name}\n{'='*60}")
    apply_preset(preset_name)
    time.sleep(2)  # 讓 PX4 套用 param

    results = []
    for i in range(runs):
        print(f"\n--- run {i+1}/{runs} ---")
        # 取起點 ground truth
        gt_start = get_gz_pose()
        log_path = f"/tmp/exp_{preset_name}_r{i+1}.log"

        # 啟動 experiment
        cmd = ['python3', '-u',
               str(Path(__file__).parent / 'forward_5m_experiment.py'),
               '--ros-args',
               '-p', 'sitl_mode:=true',
               '-p', 'drone_id:=0',          # SITL 單機 → topic 無 namespace
               '-p', f'takeoff_alt:={takeoff_alt}',
               '-p', f'forward_dist:={forward_dist}']
        env = os.environ.copy()
        env.setdefault('ROS_DOMAIN_ID', '0')

        proc = subprocess.Popen(cmd, env=env,
                                stdout=open(log_path, 'w'), stderr=subprocess.STDOUT)
        # 等實驗結束（最長 5 分鐘）
        deadline = time.time() + 300
        while proc.poll() is None and time.time() < deadline:
            time.sleep(2)
        if proc.poll() is None:
            print("  ! timeout, killing")
            proc.kill()
            proc.wait()
            force_disarm()

        # 取終點 ground truth
        gt_end = get_gz_pose()

        # 解析 log 拿 EKF 報告
        ekf_fwd, ekf_lat, ekf_alt = None, None, None
        with open(log_path) as f:
            log_content = f.read()
        m = REPORT_RE.search(log_content)
        if m:
            ekf_fwd = float(m.group(2))
            ekf_lat = float(m.group(3))
            ekf_alt = float(m.group(4))

        # 計算 ground truth 距離
        gt_dist = None
        gt_dx = gt_dy = gt_dz = None
        if gt_start and gt_end:
            gt_dx = gt_end[0] - gt_start[0]
            gt_dy = gt_end[1] - gt_start[1]
            gt_dz = gt_end[2] - gt_start[2]
            gt_dist = (gt_dx**2 + gt_dy**2 + gt_dz**2) ** 0.5

        result = {
            'preset': preset_name,
            'run': i + 1,
            'cmd_dist_m': forward_dist,
            'ekf_forward_m': ekf_fwd,
            'ekf_lateral_m': ekf_lat,
            'ekf_alt_err_m': ekf_alt,
            'gt_dx_enu': gt_dx,
            'gt_dy_enu': gt_dy,
            'gt_dz_enu': gt_dz,
            'gt_total_dist_m': gt_dist,
            'ekf_vs_cmd_err_m': (None if ekf_fwd is None else ekf_fwd - forward_dist),
            'gt_vs_cmd_err_m': (None if gt_dist is None else gt_dist - forward_dist),
            'ekf_vs_gt_err_m': (None if (ekf_fwd is None or gt_dist is None)
                                else ekf_fwd - gt_dist),
            'log_path': log_path,
        }
        print(f"  EKF fwd={ekf_fwd}  lat={ekf_lat}  alt={ekf_alt}")
        print(f"  GT  dist={gt_dist}  (dx={gt_dx}, dy={gt_dy}, dz={gt_dz})")
        results.append(result)

        # 兩輪之間 disarm + 等 EKF 穩定
        force_disarm()
        time.sleep(3)
    return results


def main():
    args = sys.argv[1:]
    runs = 1
    if '--runs' in args:
        i = args.index('--runs')
        runs = int(args[i+1])
        args = args[:i] + args[i+2:]
    presets = args if args else list(PRESETS.keys())
    out_dir = Path(__file__).parent / 'results'
    out_dir.mkdir(exist_ok=True)
    csv_path = out_dir / f"batch_{datetime.now():%Y%m%d_%H%M%S}.csv"

    all_results = []
    for p in presets:
        if p not in PRESETS:
            print(f"skip unknown preset: {p}")
            continue
        try:
            all_results += run_one_preset(p, runs=runs)
        except Exception as e:
            print(f"!! preset {p} failed: {e}")

    # 寫 CSV
    if all_results:
        keys = list(all_results[0].keys())
        with open(csv_path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            for r in all_results:
                w.writerow(r)
        print(f"\n結果寫入: {csv_path}")
        # 摘要表
        print(f"\n{'preset':<14} {'EKF fwd':>10} {'EKF lat':>10} {'GT dist':>10} {'EKF vs GT':>10}")
        for r in all_results:
            print(f"{r['preset']:<14} "
                  f"{r['ekf_forward_m'] or '---':>10} "
                  f"{r['ekf_lateral_m'] or '---':>10} "
                  f"{r['gt_total_dist_m'] or '---':>10} "
                  f"{r['ekf_vs_gt_err_m'] or '---':>10}")


if __name__ == '__main__':
    main()
