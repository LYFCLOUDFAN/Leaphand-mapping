import time
import numpy as np
from yourdfpy import URDF
import viser
from viser.extras import ViserUrdf

from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
from urdf_mapping import map_real_to_urdf

class LeapNode:
    def __init__(self):
        self.kP = 600
        self.kI = 0
        self.kD = 200
        self.curr_lim = 350
        self.prev_pos = self.pos = self.curr_pos = lhu.allegro_to_LEAPhand(np.zeros(16))
        self.motors = [i for i in range(16)]
        try:
            self.dxl_client = DynamixelClient(self.motors, '/dev/ttyUSB0', 4000000)
            self.dxl_client.connect()
        except Exception:
            self.dxl_client = DynamixelClient(self.motors, '/dev/ttyUSB1', 4000000)
            self.dxl_client.connect()

        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * 5, 11, 1)
        self.dxl_client.set_torque_enabled(self.motors, False)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kP, 84, 2)
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kP * 0.75), 84, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kI, 82, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kD, 80, 2)
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kD * 0.75), 80, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.curr_lim, 102, 2)
        
        # ➡️ 一定要初始化一下位置
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    def set_allegro(self, pose):
        pose = lhu.allegro_to_LEAPhand(pose, zeros=False)
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    def read_pos(self):
        return self.dxl_client.read_pos()

def main():
    server = viser.ViserServer()

    urdf_model = URDF.load("/home/xzx/Dro/third_party/leaphand/LEAP_Hand_API/right_hand/robot.urdf")  # ⚡换成你的路径
    viser_urdf = ViserUrdf(server, urdf_or_path=urdf_model)

    leap_hand = LeapNode()
    # 给一个初始张开姿态
    leap_hand.set_allegro(np.zeros(16))

    print("已下发初始张开命令，等待电机ready...")
    time.sleep(1.0)  # ⏳ 至少等0.5秒，让电机反应稳定

    joint_names = list(viser_urdf.get_actuated_joint_limits().keys())
    joint_order = [str(i) for i in range(16)]

    print("开始实时同步 LEAP Hand 与 VisER...")

    try:
        while True:
            real_angles = np.array(leap_hand.read_pos())  # 现在开始读取，不再timeout
            mapped_angles_dict = map_real_to_urdf(real_angles)
            mapped_angles = np.array([mapped_angles_dict[name] for name in joint_order])

            viser_urdf.update_cfg(mapped_angles)

            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n退出同步...")


if __name__ == "__main__":
    main()
