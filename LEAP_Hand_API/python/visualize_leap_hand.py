import time
import numpy as np
from yourdfpy import URDF
import viser
from viser.extras import ViserUrdf

# 启动 Viser 服务器
server = viser.ViserServer()

# 加载 URDF 文件
urdf_model = URDF.load("/home/xzx/Dro/third_party/leaphand/LEAP_Hand_API/right_hand/robot.urdf")
viser_urdf = ViserUrdf(server, urdf_or_path=urdf_model)

# 创建关节控制滑块
slider_handles = []
initial_config = []
for joint_name, (lower, upper) in viser_urdf.get_actuated_joint_limits().items():
    lower = lower if lower is not None else -np.pi
    upper = upper if upper is not None else np.pi
    initial_pos = 0.0 if lower < 0 and upper > 0 else (lower + upper) / 2.0
    slider = server.gui.add_slider(
        label=joint_name,
        min=lower,
        max=upper,
        step=1e-3,
        initial_value=initial_pos,
    )
    slider.on_update(lambda _: viser_urdf.update_cfg(
        np.array([s.value for s in slider_handles])
    ))
    slider_handles.append(slider)
    initial_config.append(initial_pos)

# 设置初始配置
viser_urdf.update_cfg(np.array(initial_config))

# 添加重置按钮
reset_button = server.gui.add_button("Reset")
@reset_button.on_click
def _(_):
    for s, init_q in zip(slider_handles, initial_config):
        s.value = init_q

# 保持服务器运行
while True:
    time.sleep(10.0)
