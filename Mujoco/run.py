# run_sim.py
import mujoco
import mujoco.viewer
from pd_controller import PDController

XML = "/Users/shinseungmin/Documents/Github/Nuri/Simulation/mujoco-3.2.0/model/nuri_4s/nuri_4s.xml"
model = mujoco.MjModel.from_xml_path(XML)
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)

# 컨트롤러 생성 (게인/목표는 원하는 값으로 조정)
controller = PDController(
    model, data,
    Kp=[20,20,20,20,20,20],
    Kd=[10,10,10,10,10,10],
    target_final=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        controller.step()
        mujoco.mj_step(model, data)
        viewer.sync()