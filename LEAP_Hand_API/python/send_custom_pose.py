#!/usr/bin/env python3
"""
send_custom_pose.py
===================

向 LEAP Hand 发送 16 维关节角度自定义动作。

用法 1：发送单帧
    python send_custom_pose.py --pose 3.124719  3.1615343 3.1523306 3.1400588 3.1308548 3.1553984 3.1492627 3.1477287 3.1308548 3.1600003 3.1538644 3.1431267 3.1584663 3.124719 3.1369908 3.1339228

用法 2：发送轨迹文件
    python send_custom_pose.py my_traj.npy --hz 15

  - 轨迹文件必须是 shape=(T, 16) 的 NumPy 文件 (单位与 LEAP 手一致)
  - --hz 控制播放频率；不指定则默认 20 Hz
"""

import argparse
import time
import numpy as np

from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu


class LeapSender:
    """极简：初始化后可多次 write_desired_pos"""

    def __init__(self, port="/dev/ttyUSB0", baud=4_000_000,
                 kp=600, kd=200, ki=0, curr_lim=350):
        self.motors = list(range(16))
        self.dxl = DynamixelClient(self.motors, port, baud)
        self.dxl.connect()

        # 基础 PID / 电流
        self.dxl.sync_write(self.motors, np.ones(16) * 5, 11, 1)       # Pos+Cur 模式
        self.dxl.set_torque_enabled(self.motors, True)
        self.dxl.sync_write(self.motors, np.ones(16) * kp, 84, 2)
        self.dxl.sync_write([0, 4, 8], np.ones(3) * kp * 0.75, 84, 2)
        self.dxl.sync_write(self.motors, np.ones(16) * kd, 80, 2)
        self.dxl.sync_write([0, 4, 8], np.ones(3) * kd * 0.75, 80, 2)
        self.dxl.sync_write(self.motors, np.ones(16) * ki, 82, 2)
        self.dxl.sync_write(self.motors, np.ones(16) * curr_lim, 102, 2)

    def send(self, pose16):
        """pose16 iterable[16] – 角度单位与 LEAP Hand API 保持一致（度）"""
        self.dxl.write_desired_pos(self.motors, np.asarray(pose16, dtype=float))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("file_or_none", nargs="?", default=None,
                        help="若给定 .npy 文件则按轨迹播放；若为空则用 --pose")
    parser.add_argument("--pose", type=float, nargs=16,
                        help="16 个角度，缺省单位=度；仅当未指定 .npy 文件时使用")
    parser.add_argument("--hz", type=float, default=20.0,
                        help="播放频率 Hz（文件模式下有效）")
    args = parser.parse_args()

    # 准备轨迹
    if args.file_or_none is not None:
        traj = np.load(args.file_or_none)          # (T,16)
        assert traj.ndim == 2 and traj.shape[1] == 16, "轨迹需为 (T,16)"
        mode = "file"
        period = 1.0 / args.hz
    else:
        assert args.pose is not None, "未提供 --pose"
        traj = np.asarray(args.pose, dtype=float).reshape(1, 16)
        mode = "single"

    sender = LeapSender()
    sender.send(traj[0])           # 先下发第一帧，防止首次超时
    time.sleep(0.5)                # 让电机就位

    if mode == "single":
        print("已发送单帧动作，开始保持姿态...（按 Ctrl+C 退出）")
        try:
            while True:
                sender.send(traj[0])
                time.sleep(0.05)  # 每 50ms 写一次，可调节
        except KeyboardInterrupt:
            print("已停止发送，程序退出。")
        return

    print(f"开始播放轨迹，共 {len(traj)} 帧，频率 {args.hz} Hz")
    t0 = time.perf_counter()
    for i, pose in enumerate(traj):
        sender.send(pose)
        nxt = t0 + (i + 1) * period
        while True:                # 精准控制周期
            now = time.perf_counter()
            if now >= nxt:
                break
            time.sleep(max(0.0, nxt - now - 0.0003))
    print("轨迹播放结束。")


if __name__ == "__main__":
    main()
