# run_sim.py
import mujoco
import mujoco.viewer
from pd_controller import PDController
import numpy as np
import matplotlib.pyplot as plt
import time


XML = "/Users/shinseungmin/Documents/Github/Mujoco/model/nuri_4s/nuri_4s.xml"
model = mujoco.MjModel.from_xml_path(XML)
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)

# 컨트롤러 생성 (게인/목표는 원하는 값으로 조정)
controller = PDController(
    model, data,
    Kp=[500,500,500,300,300,50],
    Kd=[100,100,100,100,1,1],
    target_final=[0.0, 0.0, 1.5, 0.0, 1.4, 0.0],
)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        controller.step()
        mujoco.mj_step(model, data)
        viewer.sync()