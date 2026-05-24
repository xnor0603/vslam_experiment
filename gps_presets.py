#!/usr/bin/env python3
"""GPS noise presets for PX4 SITL — 用 MAVLink 即時切換不重啟 PX4。

每個 preset 對應一種真實 GPS 接收條件，方便比較不同訊號品質下
EKF / cuVSLAM 的軌跡誤差。
"""

PRESETS = {
    # name : dict(噪音 σ + 回報 eph/epv/DOP + fix_type)
    # 所有值都用 float (PX4 內部都當 FLOAT 存)
    'rtk_fixed': {
        'desc': 'RTK Fixed — open sky w/ base station (best case)',
        'SIM_GPS_NOISE_H': 0.02, 'SIM_GPS_NOISE_V': 0.04,
        'SIM_GPS_EPH': 0.03, 'SIM_GPS_EPV': 0.05,
        'SIM_GPS_HDOP': 0.5, 'SIM_GPS_VDOP': 0.6,
        'SIM_GPS_FIXTYP': 6.0,
    },
    'rtk_float': {
        'desc': 'RTK Float — converging RTK',
        'SIM_GPS_NOISE_H': 0.10, 'SIM_GPS_NOISE_V': 0.20,
        'SIM_GPS_EPH': 0.20, 'SIM_GPS_EPV': 0.30,
        'SIM_GPS_HDOP': 0.6, 'SIM_GPS_VDOP': 0.7,
        'SIM_GPS_FIXTYP': 5.0,
    },
    'sitl_default': {
        'desc': 'PX4 SITL 原廠預設 (比真 M8N 還準)',
        'SIM_GPS_NOISE_H': 0.20, 'SIM_GPS_NOISE_V': 0.50,
        'SIM_GPS_EPH': 0.90, 'SIM_GPS_EPV': 1.78,
        'SIM_GPS_HDOP': 0.7, 'SIM_GPS_VDOP': 1.1,
        'SIM_GPS_FIXTYP': 3.0,
    },
    'm8n_open': {
        'desc': 'UBLOX M8N 開闊天空 (典型 outdoor)',
        'SIM_GPS_NOISE_H': 1.5, 'SIM_GPS_NOISE_V': 2.5,
        'SIM_GPS_EPH': 2.0, 'SIM_GPS_EPV': 3.0,
        'SIM_GPS_HDOP': 0.9, 'SIM_GPS_VDOP': 1.3,
        'SIM_GPS_FIXTYP': 3.0,
    },
    'm8n_suburban': {
        'desc': 'M8N 都市開闊 (公園邊、有低矮建物)',
        'SIM_GPS_NOISE_H': 3.0, 'SIM_GPS_NOISE_V': 5.0,
        'SIM_GPS_EPH': 4.0, 'SIM_GPS_EPV': 6.0,
        'SIM_GPS_HDOP': 1.3, 'SIM_GPS_VDOP': 2.0,
        'SIM_GPS_FIXTYP': 3.0,
    },
    'urban_canyon': {
        'desc': '都市峽谷 (高樓間、多路徑嚴重)',
        'SIM_GPS_NOISE_H': 8.0, 'SIM_GPS_NOISE_V': 15.0,
        'SIM_GPS_EPH': 10.0, 'SIM_GPS_EPV': 15.0,
        'SIM_GPS_HDOP': 3.0, 'SIM_GPS_VDOP': 5.0,
        'SIM_GPS_FIXTYP': 3.0,
    },
    'tree_cover': {
        'desc': '樹冠遮蔽 (低 signal-to-noise)',
        'SIM_GPS_NOISE_H': 5.0, 'SIM_GPS_NOISE_V': 8.0,
        'SIM_GPS_EPH': 6.0, 'SIM_GPS_EPV': 10.0,
        'SIM_GPS_HDOP': 2.0, 'SIM_GPS_VDOP': 3.0,
        'SIM_GPS_FIXTYP': 3.0,
    },
}


def apply_preset(name, mav_endpoint='udpin:0.0.0.0:14540'):
    """套用一個 preset 到 PX4 (透過 MAVLink param set)，回傳 dict of (name, set_value)"""
    from pymavlink import mavutil
    import time
    if name not in PRESETS:
        raise ValueError(f"unknown preset '{name}', choose from {list(PRESETS.keys())}")
    cfg = PRESETS[name]
    print(f"[preset:{name}] {cfg['desc']}")

    mav = mavutil.mavlink_connection(mav_endpoint)
    mav.wait_heartbeat(timeout=10)
    result = {}
    for k, v in cfg.items():
        if k == 'desc':
            continue
        ptype = (mavutil.mavlink.MAV_PARAM_TYPE_INT32
                 if isinstance(v, int) and not isinstance(v, bool)
                 else mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        mav.mav.param_set_send(mav.target_system, mav.target_component,
                               k.encode('utf-8'), float(v), ptype)
        time.sleep(0.15)
        ack = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if ack:
            result[ack.param_id.strip()] = ack.param_value
            print(f"  {ack.param_id.strip():<18} = {ack.param_value}")
        else:
            print(f"  {k:<18} no ack")
    return result


if __name__ == '__main__':
    import sys
    if len(sys.argv) < 2:
        print("Usage: gps_presets.py <preset_name>")
        print("Available presets:")
        for n, c in PRESETS.items():
            print(f"  {n:<14} - {c['desc']}")
        sys.exit(1)
    apply_preset(sys.argv[1])
