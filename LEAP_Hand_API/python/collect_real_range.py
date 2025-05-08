import numpy as np
import time
from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu

class LeapNode:
    def __init__(self):
        self.kP = 600
        self.kI = 0
        self.kD = 200
        self.curr_lim = 350
        self.prev_pos = self.pos = self.curr_pos = lhu.allegro_to_LEAPhand(np.zeros(16))
        self.motors = motors = [i for i in range(16)]
        self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB0', 4000000)
        self.dxl_client.connect()
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * 5, 11, 1)
        self.dxl_client.set_torque_enabled(motors, False)

    def read_pos(self):
        return self.dxl_client.read_pos()

def main():
    leap_hand = LeapNode()

    min_values = np.full(16, np.inf)
    max_values = np.full(16, -np.inf)

    print("开始采集关节极值，请手动动一动手指，观察角度变化。按 Ctrl+C 结束采集。")

    try:
        while True:
            pos = np.array(leap_hand.read_pos()) 
            min_values = np.minimum(min_values, pos)
            max_values = np.maximum(max_values, pos)

            print("实时关节角度: ", np.round(pos, 2))
            print("当前已记录最小值: ", np.round(min_values, 2))
            print("当前已记录最大值: ", np.round(max_values, 2))
            print("-" * 80)
            time.sleep(0.1)  # 每0.1秒刷新一次
    except KeyboardInterrupt:
        print("\n采集结束")
        print("最终关节最小值：", np.round(min_values, 2))
        print("最终关节最大值：", np.round(max_values, 2))

if __name__ == "__main__":
    main()
