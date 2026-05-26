#!/usr/bin/env python3
"""PX4 EKF2-only baseline 實驗：起飛 → 懸停 → 前進 5 m → 懸停 → 降落。

底層邏輯：在不啟動 isaac_ros_visual_slam 的情況下，PX4 EKF2 只融合 GPS+IMU+Mag。
本腳本只負責「下指令」+「記錄真值（指令點/EKF 估計/GPS 原始）」，不做估計。

執行流程：
    階段       時間軸     動作
    INIT       0–3s       等待 EKF ref + 健康檢查 + 鎖定 takeoff 位姿
    ARM        ~5s        Offboard heartbeat → ARM
    TAKEOFF    +T_climb   爬升至 takeoff_alt
    HOVER1     +T_hover   原地懸停讓 EKF 收斂
    FORWARD    +T_move    沿 takeoff_heading 方向走 forward_dist m（setpoint leash 平滑）
    HOVER2     +T_hover   抵達後懸停確認終點
    LAND       —          PX4 自帶 LAND 模式

執行：
    # 1. 確認 PX4 EKF2 沒在融合 vision（EKF2_AID_MASK 不含 bit3）
    # 2. 終端 A：錄 bag
    ros2 bag record -o /home/landis/VSLAM/isaac_ros-dev/bags/ekf2_baseline_$(date +%Y%m%d_%H%M%S) \\
        /px4_1/fmu/out/vehicle_local_position \\
        /px4_1/fmu/out/vehicle_gps_position \\
        /px4_1/fmu/out/vehicle_status \\
        /px4_1/fmu/in/trajectory_setpoint
    # 3. 終端 B：跑這隻
    python3 forward_5m_experiment.py --ros-args -p drone_id:=1 -p takeoff_alt:=2.0 -p forward_dist:=5.0
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleStatus, VehicleLocalPosition)


class ForwardExperiment(Node):
    def __init__(self):
        super().__init__('forward_5m_experiment')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('drone_id', 1)
        self.declare_parameter('sitl_mode', False)
        self.declare_parameter('takeoff_alt', 2.0)       # m above takeoff
        self.declare_parameter('forward_dist', 5.0)      # m along takeoff heading
        self.declare_parameter('cruise_speed', 0.5)      # m/s setpoint leash speed
        self.declare_parameter('hover_time', 5.0)        # s 懸停穩定時間
        self.declare_parameter('reach_thresh', 0.4)      # m 抵達判定半徑

        self.drone_id     = self.get_parameter('drone_id').value
        self.sitl_mode    = self.get_parameter('sitl_mode').value
        self.takeoff_alt  = float(self.get_parameter('takeoff_alt').value)
        self.forward_dist = float(self.get_parameter('forward_dist').value)
        self.cruise_speed = float(self.get_parameter('cruise_speed').value)
        self.hover_time   = float(self.get_parameter('hover_time').value)
        self.reach_thresh = float(self.get_parameter('reach_thresh').value)
        # drone_id=0 → SITL 單機模式，topic 無 namespace 前綴（/fmu/out/...）
        # drone_id>0 → 多機 / 真機，topic 走 /px4_<id>/fmu/...
        if self.drone_id == 0:
            self.ns = 'sitl'
            topic_prefix = ''
        else:
            self.ns = f'px4_{self.drone_id}'
            topic_prefix = f'/{self.ns}'

        # ── QoS（與 real_drone_node 對齊）──────────────────────────────
        durability = DurabilityPolicy.TRANSIENT_LOCAL if self.sitl_mode else DurabilityPolicy.VOLATILE
        self.px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                  durability=durability,
                                  history=HistoryPolicy.KEEP_LAST, depth=1)

        # ── PX4 IO ──────────────────────────────────────────────────────
        self.pub_ocm  = self.create_publisher(OffboardControlMode, f'{topic_prefix}/fmu/in/offboard_control_mode', self.px4_qos)
        self.pub_traj = self.create_publisher(TrajectorySetpoint,  f'{topic_prefix}/fmu/in/trajectory_setpoint',   self.px4_qos)
        self.pub_cmd  = self.create_publisher(VehicleCommand,      f'{topic_prefix}/fmu/in/vehicle_command',       self.px4_qos)
        self.create_subscription(VehicleStatus,        f'{topic_prefix}/fmu/out/vehicle_status',         self.cb_status,    self.px4_qos)
        self.create_subscription(VehicleLocalPosition, f'{topic_prefix}/fmu/out/vehicle_local_position', self.cb_local_pos, self.px4_qos)

        # ── State ───────────────────────────────────────────────────────
        self.phase = 'INIT'
        self.tick = 0
        self.phase_tick = 0
        self.armed = False
        self.preflight_ok = False
        self.system_id = 1
        self.cur_xyz = [0.0, 0.0, 0.0]
        self.cur_heading = 0.0
        self.local_ready = False
        self.ground_z = None      # 第一筆 local_pos 的 z，當作地面參考算相對高度

        # 起飛時鎖定的位姿（forward 方向的物理錨點）
        self.takeoff_x = None
        self.takeoff_y = None
        self.takeoff_heading = None
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_yaw = float('nan')

        self.timer = self.create_timer(0.05, self.tick_cb)  # 20 Hz
        self.get_logger().info(
            f'[{self.ns}] 實驗就緒：takeoff_alt={self.takeoff_alt}m, '
            f'forward={self.forward_dist}m, cruise={self.cruise_speed}m/s')

    # ── Callbacks ───────────────────────────────────────────────────────
    def cb_status(self, msg: VehicleStatus):
        self.armed = (int(msg.arming_state) == 2)
        self.preflight_ok = bool(msg.pre_flight_checks_pass)
        if hasattr(msg, 'system_id') and msg.system_id > 0:
            self.system_id = int(msg.system_id)

    def cb_local_pos(self, msg: VehicleLocalPosition):
        self.cur_xyz = [float(msg.x), float(msg.y), float(msg.z)]
        self.cur_heading = float(msg.heading)
        if self.ground_z is None:
            self.ground_z = float(msg.z)
        self.local_ready = bool(msg.xy_valid and msg.z_valid)

    # ── PX4 Publishers ──────────────────────────────────────────────────
    def send_ocm(self):
        m = OffboardControlMode()
        m.position = True
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_ocm.publish(m)

    def send_setpoint(self, x, y, z, yaw):
        # Setpoint leash：把 setpoint 限制在 cruise_speed*lookahead 內，PX4 會以 cruise_speed 跟上
        lookahead = 2.0  # s
        leash = self.cruise_speed * lookahead
        cx, cy, cz = self.cur_xyz
        dx, dy, dz = x - cx, y - cy, z - cz
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        if dist > leash and dist > 1e-3:
            s = leash / dist
            x, y, z = cx + dx*s, cy + dy*s, cz + dz*s
        m = TrajectorySetpoint()
        m.position = [float(x), float(y), float(z)]
        m.yaw = float(yaw)
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_traj.publish(m)

    def send_vehicle_cmd(self, cmd, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.command = cmd
        m.param1 = float(p1); m.param2 = float(p2)
        m.target_system = self.system_id
        m.target_component = 1
        m.source_system = 1; m.source_component = 1
        m.from_external = True
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_cmd.publish(m)

    # ── Phase Helpers ───────────────────────────────────────────────────
    def goto(self, phase):
        self.get_logger().info(f'[{self.ns}] {self.phase} → {phase}')
        self.phase = phase
        self.phase_tick = 0

    def horiz_dist_to_target(self):
        cx, cy, _ = self.cur_xyz
        return math.hypot(cx - self.target_x, cy - self.target_y)

    # ── Main Loop（20 Hz）───────────────────────────────────────────────
    def tick_cb(self):
        self.tick += 1
        self.phase_tick += 1
        t_phase = self.phase_tick * 0.05

        # 全程必須持續發 OCM heartbeat 與當前 setpoint，否則 PX4 會踢出 Offboard
        # LAND 階段必須停止發 OFFBOARD 心跳，讓 PX4 切到 AUTO.LAND 安全降落
        if self.phase not in ('INIT', 'LAND'):
            self.send_ocm()
            self.send_setpoint(self.target_x, self.target_y, self.target_z, self.target_yaw)

        # 每秒在同一行顯示當前位置（stdout，不干擾 ROS log 的 stderr）
        if self.tick % 20 == 0:
            x, y, z = self.cur_xyz
            alt = -(z - self.ground_z) if self.ground_z is not None else 0.0
            print(f'\r  [pos] x={x:+.2f}  y={y:+.2f}  alt={alt:+.2f}m  phase={self.phase}    ',
                  end='', flush=True)

        # ─── INIT：等 EKF + Preflight ──────────────────────────────────
        # SITL 中 preflight_ok 永遠 false（沒 RC），改為「local_ready 後再等 EKF 收斂 30s」
        sitl_min_wait = 30.0
        if self.phase == 'INIT':
            if t_phase > 90.0:
                self.get_logger().error('90s 內未拿到 local_pos / preflight，放棄')
                raise SystemExit(1)
            sitl_ekf_ready = self.local_ready and t_phase >= sitl_min_wait
            real_ready    = self.local_ready and self.preflight_ok
            # SITL 模式：強制等 EKF 收斂，不管 preflight_ok（COM_RC_IN_MODE=1 會讓它過早變 true）
            # Real 模式：用原本 preflight 邏輯
            ready = sitl_ekf_ready if self.sitl_mode else real_ready
            if ready:
                # 鎖定起飛錨點
                self.takeoff_x = self.cur_xyz[0]
                self.takeoff_y = self.cur_xyz[1]
                self.takeoff_heading = self.cur_heading
                self.target_x = self.takeoff_x
                self.target_y = self.takeoff_y
                self.target_z = self.cur_xyz[2]            # 先鎖在當前高度
                self.target_yaw = self.takeoff_heading      # lock yaw
                self.get_logger().info(
                    f'[{self.ns}] takeoff anchor: pos=({self.takeoff_x:.2f},{self.takeoff_y:.2f},{self.cur_xyz[2]:.2f}) '
                    f'heading={math.degrees(self.takeoff_heading):.1f}°')
                self.goto('ARM')
            return

        # ─── ARM：retry loop 風格（對齊 real_drone_node.perform_safety_takeoff）
        # tick 1~20  : 持續發 OCM+SP 同時每 tick 嘗試切 OFFBOARD（最多送 20 次）
        # tick 21~40 : 持續發 OCM+SP 同時每 tick 嘗試 ARM（最多送 20 次）
        if self.phase == 'ARM':
            if 1 <= self.phase_tick <= 20:
                self.send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            if 21 <= self.phase_tick <= 40 and not self.armed:
                self.send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            if self.phase_tick > 40 and self.armed:
                self.target_z = self.cur_xyz[2] - self.takeoff_alt  # NED z 負 = 上
                self.goto('TAKEOFF')
            elif self.phase_tick > 200:  # 10s 都沒 ARM 放棄
                self.get_logger().error('Arm 超時')
                raise SystemExit(1)
            return

        # ─── TAKEOFF：爬升至 takeoff_alt ──────────────────────────────
        if self.phase == 'TAKEOFF':
            alt = -(self.cur_xyz[2] - self.target_z)  # 已爬升量
            if abs(alt) < 0.3 or t_phase > 30.0:
                self.get_logger().info(f'[{self.ns}] 抵達高度 alt_err={alt:.2f}m')
                self.goto('HOVER1')
            return

        # ─── HOVER1：懸停穩 EKF ───────────────────────────────────────
        if self.phase == 'HOVER1':
            if t_phase >= self.hover_time:
                # 計算前進終點（沿 takeoff_heading 方向）
                self.target_x = self.takeoff_x + self.forward_dist * math.cos(self.takeoff_heading)
                self.target_y = self.takeoff_y + self.forward_dist * math.sin(self.takeoff_heading)
                self.get_logger().info(
                    f'[{self.ns}] 開始前進 → ({self.target_x:.2f}, {self.target_y:.2f}, {self.target_z:.2f})')
                self.goto('FORWARD')
            return

        # ─── FORWARD：直線飛 forward_dist ─────────────────────────────
        if self.phase == 'FORWARD':
            d = self.horiz_dist_to_target()
            if self.phase_tick % 20 == 0:
                cx, cy, cz = self.cur_xyz
                self.get_logger().info(f'[{self.ns}] FORWARD pos=({cx:.2f},{cy:.2f},{cz:.2f}) dist_remain={d:.2f}')
            if d < self.reach_thresh:
                self.goto('HOVER2')
            elif t_phase > self.forward_dist / max(self.cruise_speed, 0.1) * 3.0 + 10.0:
                self.get_logger().warn(f'[{self.ns}] FORWARD 超時，仍距 {d:.2f}m，強制收尾')
                self.goto('HOVER2')
            return

        # ─── HOVER2：終點懸停 ─────────────────────────────────────────
        if self.phase == 'HOVER2':
            if t_phase >= self.hover_time:
                cx, cy, cz = self.cur_xyz
                # 量化結果報告
                ex = cx - self.takeoff_x
                ey = cy - self.takeoff_y
                forward_actual = ex * math.cos(self.takeoff_heading) + ey * math.sin(self.takeoff_heading)
                lateral_actual = -ex * math.sin(self.takeoff_heading) + ey * math.cos(self.takeoff_heading)
                self.get_logger().info(
                    f'[{self.ns}] === 終點報告（EKF 估計）===\n'
                    f'  指令前進距離     : {self.forward_dist:.3f} m\n'
                    f'  EKF 沿 heading  : {forward_actual:.3f} m  (誤差 {forward_actual - self.forward_dist:+.3f} m)\n'
                    f'  EKF 側向漂移     : {lateral_actual:+.3f} m\n'
                    f'  EKF 高度誤差     : {-(cz - (self.cur_xyz[2] - 0)):.3f} m')
                self.goto('LAND')
            return

        # ─── LAND ──────────────────────────────────────────────────────
        if self.phase == 'LAND':
            if self.phase_tick == 1:
                self.send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            if not self.armed and self.phase_tick > 40:
                self.get_logger().info(f'[{self.ns}] 已上鎖，實驗結束')
                raise SystemExit(0)
            elif self.phase_tick > 600:  # 30s
                self.get_logger().warn('LAND 超時，強制 disarm')
                self.send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
                raise SystemExit(0)


def main():
    rclpy.init()
    node = ForwardExperiment()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        try: rclpy.shutdown()
        except Exception: pass


if __name__ == '__main__':
    main()
