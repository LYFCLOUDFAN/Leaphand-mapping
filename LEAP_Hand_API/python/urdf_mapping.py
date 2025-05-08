"""
urdf_mapping.py

把LEAP Hand实际读取到的关节角度，映射到URDF的可视化角度。
"""

import numpy as np

# 1. 关节编号到URDF关节名字的对应关系
JOINT_ID_TO_URDF_NAME = {
    0: "0",
    1: "1",
    2: "2",
    3: "3",
    4: "4",
    5: "5",
    6: "6",
    7: "7",
    8: "8",
    9: "9",
    10: "10",
    11: "11",
    12: "12",
    13: "13",
    14: "14",
    15: "15",
}

# 2. 实测硬件关节角度范围,注意对于0，4，8，12的range需要截去90度，或者乘上0.75（在后面已经处理这里不需要更改），关于数据13按照visualize实测会更好，因为13实际range太大。
REAL_JOINT_LIMITS = {
    0: (1.56, 4.52),
    1: (2.81, 5.31),
    2: (2.69, 5.12),
    3: (2.79, 5.19),
    4: (1.50, 4.50),
    5: (2.81, 5.30),
    6: (2.71, 4.48),
    7: (2.77, 5.19),
    8: (1.59, 4.51),
    9: (2.87, 5.34),
    10: (2.62, 5.06),
    11: (2.77, 5.18),
    12: (2.26, 5.35),
    13: (2.45, 5.45),
    14: (1.83, 4.96),
    15: (1.82, 5.06),
}
#bias 0.3925
#3.0805 3.08 3.235 3.148 3.0625 3.07 3.216 3.148 3.0705 3.11 3.175 3.138 3.0015 3.007 3.06 3.18

# 3. URDF定义的关节角度范围
URDF_JOINT_LIMITS = {
    0: (-1.047, 1.047),
    1: (-0.314, 2.23),
    2: (-0.506, 1.885),
    3: (-0.366, 2.042),
    4: (-1.047, 1.047),
    5: (-0.314, 2.23),
    6: (-0.506, 1.885),
    7: (-0.366, 2.042),
    8: (-1.047, 1.047),
    9: (-0.314, 2.23),
    10: (-0.506, 1.885),
    11: (-0.366, 2.042),
    12: (-0.349, 2.094),
    13: (-0.47, 2.443),
    14: (-1.20, 1.90),
    15: (-1.34, 1.88),
}

def map_real_to_urdf(real_joint_angles):
    """
    输入真实LEAP Hand读到的16个关节角度，输出映射后的URDF角度。
    
    参数：
        real_joint_angles (list or np.ndarray): shape = (16,)

    返回：
        urdf_joint_angles (dict): 关节名到角度的映射
    """
    urdf_joint_angles = {}
    SPECIAL_JOINTS = {0, 4, 8, 12}
    for idx in range(16):
        real_min, real_max = REAL_JOINT_LIMITS[idx]
        urdf_min, urdf_max = URDF_JOINT_LIMITS[idx]

        if idx in SPECIAL_JOINTS:
            normalized = (real_joint_angles[idx] - real_min) / ((real_max - real_min) * 0.75)
            normalized = np.clip(normalized, 0.0, 1.0)

        # 归一化真实角度到 [0, 1]
        normalized = (real_joint_angles[idx] - real_min) / (real_max - real_min)
        normalized = np.clip(normalized, 0.0, 1.0)

        # 映射到URDF角度区间
        urdf_angle = urdf_min + normalized * (urdf_max - urdf_min)
        urdf_joint_angles[JOINT_ID_TO_URDF_NAME[idx]] = urdf_angle

    return urdf_joint_angles

def map_urdf_to_real(urdf_joint_angles):
    """
    将 URDF 中的16个关节角度映射为 LEAP Hand 控制所需的真实角度值。

    参数：
        urdf_joint_angles (list or np.ndarray): shape = (16,)，单位：rad

    返回：
        real_joint_angles (np.ndarray): shape = (16,)，单位：与 REAL_JOINT_LIMITS 一致（rad 或 degree）
    """
    real_joint_angles = []
    SPECIAL_JOINTS = {0, 4, 8, 12}

    for idx in range(16):
        urdf_min, urdf_max = URDF_JOINT_LIMITS[idx]
        real_min, real_max = REAL_JOINT_LIMITS[idx]

        # 归一化 URDF 角度到 [0, 1]
        normalized = (urdf_joint_angles[idx] - urdf_min) / (urdf_max - urdf_min)
        normalized = np.clip(normalized, 0.0, 1.0)

        if idx in SPECIAL_JOINTS:
            # 这几个关节在 map_real_to_urdf() 中使用了 0.75 的缩放，这里需要反缩放
            normalized /= 1
            normalized = np.clip(normalized, 0.0, 1.0)

        # 映射回真实角度
        real_angle = real_min + normalized * (real_max - real_min)
        real_joint_angles.append(real_angle)

    return np.array(real_joint_angles)
