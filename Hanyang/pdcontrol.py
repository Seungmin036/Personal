import mujoco
import mujoco.viewer
import numpy as np
import time

# --- 1. 모델 및 데이터 로드 ---
model_path = '/Users/shinseungmin/Documents/Github/Hanyang/model/franka_emika_panda/mjx_scene.xml'
try:
    model = mujoco.MjModel.from_xml_path(model_path)
except Exception as e:
    print(f"모델 로딩 오류: {e}")
    exit()

data = mujoco.MjData(model)
viewer = mujoco.viewer.launch_passive(model, data)

# --- 2. 제어기 파라미터 설정 ---
num_joints = model.nu

# 목표 자세 설정 (예: 약간 구부린 자세)
# 로봇의 관절 수에 맞게 설정해야 합니다. Nuri 로봇은 6개.
target_qpos = np.array([0.1, 0.1, 0.01, 0.01, 0.01, 0.01, 0.01, 0.04, 0.04])  # 7자유도 로봇의 예시

# 목표 속도는 0으로 가정합니다 (정지 상태).
target_qvel = np.zeros(num_joints)

# PD 제어기 이득(gain) 설정
# 이 값들을 조정하여 로봇의 반응 속도와 안정성을 바꿀 수 있습니다.
kp = np.array([1000, 1000, 750, 750, 300, 300, 300, 100, 100])  # 비례 이득 (P gain)
kd = np.array([20,20,4,4,2,2,2,1,1])  # 미분 이득 (D gain)

# --- 3. 시뮬레이션 초기화 ---
# 시뮬레이션 시작 시 로봇을 초기 자세로 설정
initial_qpos = np.zeros(num_joints)
data.qpos[:] = initial_qpos
first_run = True  # 첫 번째 루프를 위한 플래그
step = 0

# 궤적 생성 관련 변수
duration = 5.0  # 목표까지 이동하는 데 걸리는 시간 (초)
elapsed_time = 0.0

# --- 4. 시뮬레이션 루프 ---
try:
    while viewer.is_running():
        step_start = time.time()
        
        # 경과 시간 업데이트
        elapsed_time += model.opt.timestep

        # --- ✨ 선형 보간(Linear Interpolation)으로 중간 목표 생성 ✨ ---
        # 0(시작)과 1(끝) 사이의 진행률 계산
        alpha = min(elapsed_time / duration, 1.0)
        
        # 중간 목표 지점 계산
        interpolated_target_qpos = initial_qpos + alpha * (target_qpos - initial_qpos)
        
        # 중간 목표 속도는 (아주 간단하게) 0으로 가정
        interpolated_target_qvel = np.zeros(num_joints)

        # --- 순수 PD 제어 로직 ---
        position_error = interpolated_target_qpos - data.qpos
        velocity_error = interpolated_target_qvel - data.qvel
        
        pd_torque = kp * position_error + kd * velocity_error
        data.ctrl[:] = pd_torque

        mujoco.mj_step(model, data)

        viewer.sync()
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

except Exception as e:
    print(f"시뮬레이션 중 오류 발생: {e}")
finally:
    if viewer and viewer.is_running():
        viewer.close()
        


