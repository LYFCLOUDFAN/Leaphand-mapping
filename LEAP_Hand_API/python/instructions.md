## Welcome to the LEAP Hand Python SDK
####下载Dynamixel wizard####
#### Install On Ubuntu
- `python -m venv test_env`
- `source test_env/bin/activate`
- `pip install dynamixel_sdk numpy`
- `python main.py`

#### Install On Windows
- Use Windows powershell
- We recommend to create a virtual environment and pip install the code:
- `Set-ExecutionPolicy Unrestricted -Scope Process`
- `python -m venv test_env`
- `.\test_env\Scripts\activate.ps1`
- `pip install dynamixel_sdk numpy`
- `python main.py`

Please see main.py for further details.  It should be easy to read.  :)

#### 下载Dynamixel wizard 2.0(optional) ####

#### 使能 ####
- sudo chmod 666 /dev/ttyUSB2

#### visualize_urdf(official)###
- python visualize_leap_hand.py

#### collect_real_range/compute bias
- python collect_real_range.py

#### mapping ####
- python urdf_mapping.py

#### visualize realtime(after mapping)###
- python visualize_leap_hand_realtime.py 

"""
send_custom_pose.py
===================

向 LEAP Hand 发送 16 维关节角度自定义动作。

用法 1：发送单帧
    python send_custom_pose.py --pose 3.124719  3.1615343 3.1523306 3.1400588 3.1308548 3.1553984 3.1492627 3.1477287 3.1308548 3.1600003 3.1538644 3.1431267 3.1584663 3.124719 3.1369908 3.1339228

    python send_custom_pose.py --pose 3.1273 2.8100 4.7512 3.8787 3.7473 3.7872 4.1290 2.9599 4.3990 3.9145 4.7462 2.7700 4.5365 -2.4671 3.3332 4.1665

用法 2：发送轨迹文件
    python send_custom_pose.py my_traj.npy --hz 15

  - 轨迹文件必须是 shape=(T, 16) 的 NumPy 文件 (单位与 LEAP 手一致)
  - --hz 控制播放频率；不指定则默认 20 Hz
"""