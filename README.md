# Experiment Kit — PX4 EKF2 / cuVSLAM 對照實驗

六個腳本四件事：(1) 控飛機飛 5 m、(2) 修補 cuVSLAM 的 TF 鏈、(3) 在 SITL 中切換不同 GPS 品質批跑、(4) 後處理畫軌跡。

---

## 檔案清單

| 檔案 | 角色 | 部署位置（拉到新機器後該放哪） |
|---|---|---|
| `forward_5m_experiment.py` | 起飛指令節點：起飛 → 懸停 → 前進 5 m → 懸停 → 降落（支援真機與 SITL） | 任意位置可跑；要 `ros2 run` 就丟進某個 ROS 2 Python pkg |
| `gps_presets.py` | 7 個 GPS 噪音 preset，用 MAVLink 即時切換不需要重啟 PX4 | 任意位置；單獨跑可當 CLI 用 |
| `batch_run_presets.py` | 自動跑多個 preset × 多次，採樣 Gazebo ground truth，輸出 CSV 與摘要表 | 任意位置（SITL 主機上） |
| `plot_three_trajectories.py` | 三源軌跡圖：vehicle_local_position / vehicle_gps_position / VSLAM odometry | 任意位置 |
| `plot_single_trajectory.py` | 單源 fallback（bag 只有一個位置 topic 時用） | 任意位置 |
| `isaac_ros_visual_slam_bag.launch.py` | cuVSLAM bag-replay 模式專用 launch，已補 4 個 static_transform_publisher 解 `camera_infra1_optical_frame not found` | **必須**放回 `isaac_ros_visual_slam/isaac_ros_visual_slam/launch/`，再 `colcon build` |

執行產生的東西會落在 `results/`（`batch_<timestamp>.csv`、軌跡 png）。

---

## 1. 飛行端 — `forward_5m_experiment.py`

### 前置條件（PX4 飛控側設定，跟著飛控板走）

| 參數 | 值 | 為何 |
|---|---|---|
| `EKF2_AID_MASK` | **不含** bit 3 (vision pos)、bit 4 (vision yaw) | 確保純 EKF2 baseline，不偷融 VSLAM |
| `COM_RCL_EXCEPT` | 允許 Offboard | 不然 RC 一斷就掉 Offboard |

### 模式 A — 直接 python3（最快）

```bash
source /opt/ros/humble/setup.bash
source ~/<your_ros2_ws>/install/setup.bash   # 確保 px4_msgs 在環境裡
python3 forward_5m_experiment.py --ros-args \
    -p drone_id:=1 -p takeoff_alt:=2.0 -p forward_dist:=5.0 -p cruise_speed:=0.5
```

### 模式 B — `ros2 run`

把這隻 `.py` 丟進任何 ROS 2 Python pkg（例如 `offboard_control_pkg`）的 `<pkg>/<pkg>/` 下，
`<pkg>/setup.py` 的 `entry_points -> console_scripts` 加一行：

```python
'forward_5m = <pkg>.forward_5m_experiment:main',
```

然後 build + run：

```bash
cd ~/<your_ros2_ws>
colcon build --packages-select <pkg> --symlink-install
source install/setup.bash
ros2 run <pkg> forward_5m --ros-args -p drone_id:=1 -p forward_dist:=5.0
```

### 模式 C — SITL（Gazebo）

SITL 中沒有 RC，`pre_flight_checks_pass` 可能太早變 true，會在 EKF 還沒收斂時就 ARM。
打開 `sitl_mode:=true`：QoS 切 `TRANSIENT_LOCAL` 對齊 PX4 SITL bridge，INIT 階段改成
「拿到 `local_ready` 後再強制等 30 s」，忽略 `pre_flight_checks_pass`。

```bash
python3 forward_5m_experiment.py --ros-args -p sitl_mode:=true \
    -p drone_id:=1 -p takeoff_alt:=2.0 -p forward_dist:=5.0
```

### 可調參數

| 參數 | 預設 | 說明 |
|---|---|---|
| `drone_id` | 1 | 對應 `/px4_<id>/fmu/...` namespace |
| `sitl_mode` | False | True 時切 TRANSIENT_LOCAL QoS、INIT 改用強制 30 s EKF 等待（見模式 C） |
| `takeoff_alt` | 2.0 m | 起飛離地高度 |
| `forward_dist` | 5.0 m | 沿 takeoff heading 方向走多少 |
| `cruise_speed` | 0.5 m/s | setpoint leash 速度（PX4 會以這個速度跟上） |
| `hover_time` | 5.0 s | 起飛/終點懸停時間（讓 EKF 穩） |
| `reach_thresh` | 0.4 m | 視為抵達的水平距離門檻 |

### 終點報告（被 batch 抓的格式）

實驗結束會 log 一段「終點報告」，含「指令前進距離 / EKF 沿 heading / EKF 側向漂移 / EKF 高度誤差」四個數字。
`batch_run_presets.py` 用 regex 解這段，**改文字格式時記得同步改 regex**。

### 同時錄 bag（真機追過程的證據）

另開終端：

```bash
ros2 bag record -o ekf2_baseline_$(date +%Y%m%d_%H%M%S) \
    /px4_1/fmu/out/vehicle_local_position \
    /px4_1/fmu/out/vehicle_gps_position \
    /px4_1/fmu/out/vehicle_status \
    /px4_1/fmu/in/trajectory_setpoint
```

---

## 2. GPS noise sweep — `gps_presets.py` + `batch_run_presets.py`

### preset 一覽（`gps_presets.py`）

每個 preset 是一組 `SIM_GPS_NOISE_H/V`、`SIM_GPS_EPH/EPV`、`SIM_GPS_HDOP/VDOP`、`SIM_GPS_FIXTYP` 的組合，透過 MAVLink `PARAM_SET` 即時下發（不用重啟 PX4）。

| preset | 對應場景 |
|---|---|
| `m8n_open` | UBLOX M8N 開闊天空（典型 outdoor） |
| `m8n_suburban` | 都市開闊（公園、低矮建物） |
| `tree_cover` | 樹冠遮蔽（低 SNR） |
| `urban_canyon` | 都市峽谷（高樓間、多路徑嚴重） |

只保留無 RTK 的真 M8N 等級檔位 — 我們要看的是 EKF2 在現實 GPS 品質下會差多少，RTK / SITL default 那種準度沒參考價值。

單獨用作 CLI：

```bash
python3 gps_presets.py m8n_open         # 套用一個 preset
python3 gps_presets.py                  # 不帶參數列出全部
```

### 批跑 — `batch_run_presets.py`

對每個 preset：套 preset → 採 Gazebo ground truth 起點 → 啟動 `forward_5m_experiment.py`（`sitl_mode:=true`） → 採終點 ground truth → regex 解 log 拿 EKF 報告 → 結果寫一列 CSV。

```bash
# 跑全部 preset，各 1 次
python3 batch_run_presets.py

# 只跑指定的（白名單），各 1 次
python3 batch_run_presets.py m8n_open urban_canyon

# 對 m8n_open 跑 5 次（CSV 會有 5 列）
python3 batch_run_presets.py --runs 5 m8n_open
```

CSV 落在 `results/batch_<YYYYMMDD_HHMMSS>.csv`，欄位：

```
preset, run, cmd_dist_m,
ekf_forward_m, ekf_lateral_m, ekf_alt_err_m,
gt_dx_enu, gt_dy_enu, gt_dz_enu, gt_total_dist_m,
ekf_vs_cmd_err_m, gt_vs_cmd_err_m, ekf_vs_gt_err_m,
log_path
```

注意事項：

- ground truth 透過 `gz model --model x500_0 --pose` 抓，預設機體名 `x500_0`，多機要改 `get_gz_pose()` 參數。
- MAVLink endpoint 預設 `udpin:0.0.0.0:14540`（PX4 SITL 一般 GCS port），跟你 PX4 起飛時的設定要對得上。
- 每個 run 最長等 300 s，超時就 kill + force_disarm；下一輪起跑前 sleep 3 s 等 EKF 重新收斂。

---

## 3. cuVSLAM 端 — `isaac_ros_visual_slam_bag.launch.py`

### 為什麼需要這支

bag-replay 模式下 RealSense driver 沒在跑 → TF 樹缺 `camera_infra1_optical_frame / camera_infra2_optical_frame / camera_imu_optical_frame` → cuVSLAM 直接報錯不工作。

這支 launch 在原 `visual_slam_node` 之前加了 4 個 `static_transform_publisher` 把 TF 鏈補完整：

```
base_link → camera_link → {infra1, infra2, imu}_optical_frame
```

數值依 RealSense D435i 的 URDF（infra2 y=-0.050、IMU 偏移、optical frame RPY=(-π/2,0,-π/2)）。

### 部署

```bash
cp isaac_ros_visual_slam_bag.launch.py \
   <your_workspace>/src/isaac_ros_visual_slam/isaac_ros_visual_slam/launch/

cd <your_workspace>
colcon build --packages-select isaac_ros_visual_slam --symlink-install
source install/setup.bash
```

### 跑法

```bash
# 終端 A：跑 cuVSLAM
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_bag.launch.py use_sim_time:=True

# 終端 B：重播 bag（記得 --clock）
ros2 bag play <your_bag> --clock

# 終端 C：錄 cuVSLAM 輸出
ros2 bag record -o vslam_output_$(date +%Y%m%d_%H%M%S) \
    /visual_slam/tracking/odometry \
    /visual_slam/tracking/vo_pose \
    /visual_slam/tracking/slam_path \
    /visual_slam/status
```

---

## 4. 分析端 — `plot_three_trajectories.py` / `plot_single_trajectory.py`

### 三源軌跡圖（推薦，PX4 + GPS + VSLAM 都有時用）

```bash
python3 plot_three_trajectories.py <source_bag_dir_or_db3> <vslam_bag_dir_or_db3>
```

- 三 panel：`vehicle_local_position` / `vehicle_gps_position` / `/visual_slam/tracking/odometry`
- VSLAM panel 已 invert x 軸對齊物理視角
- 圖檔：`<OUT 目錄>/<bag起始時間>_trajectories.png`

> 注意：腳本內 `DEFAULT_TRIM` / `DEFAULT_VSLAM` 寫死了原機器的舊 bag 路徑，新機器上**一定要傳 CLI 參數**，或改 default。
> 另外 topic 寫死成 `/px4_2/fmu/...`，drone_id 不是 2 就要手動改。

### 單源 fallback（只錄到一個位置 topic 時用）

```bash
python3 plot_single_trajectory.py <bag_dir_or_db3>
```

自動偵測 `SensorGps` / `VehicleLocalPosition` / `Odometry` 中第一個出現的，畫對應 panel。

### 輸出目錄

兩支腳本目前都寫死輸出到 `/home/landis/VSLAM/ClaudeCode/figures/` — 在新機器上**先改路徑**，不然會找不到資料夾。（TODO：之後改成輸出到 `results/figures/`，跟 batch 走同一個輸出根目錄。）

---

## 實驗 SOP（真機，一次走完的閉環）

| 步驟 | 動作 | 證據 |
|---|---|---|
| 1 | 地面用粉筆/膠帶標起點 A，皮尺往機頭方向拉 5 m 標 B 點 | 拍照 |
| 2 | 確認 PX4 `EKF2_AID_MASK` 不含 vision bits | QGC 截圖 |
| 3 | 終端 A 起 `ros2 bag record` | bag 路徑 |
| 4 | 終端 B 跑 `forward_5m_experiment.py` | log 報告 |
| 5 | 落地，皮尺量降落點到 B 點偏差 | 寫實驗記錄卡 |
| 6 | 跑 `plot_three_trajectories.py` 出圖 | png |

實驗記錄卡欄位：日期 / 場地 / 天氣 / 起飛 sat & eph / 指令距離 / EKF 估計距離（log）/ **物理皮尺距離** / **物理側向偏差** / 異常記錄。

## SITL SOP（GPS sweep）

| 步驟 | 動作 |
|---|---|
| 1 | 起 PX4 SITL + Gazebo（`x500_0`） |
| 2 | `python3 batch_run_presets.py`（或挑 preset 子集） |
| 3 | 看 `results/batch_*.csv`，比 `ekf_vs_gt_err_m`、`ekf_vs_cmd_err_m` |
