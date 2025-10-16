import mujoco
import mujoco.viewer
import numpy as np
import time

# --- 1. 모델 및 데이터 로드 ---
model_path = '/Users/shinseungmin/Documents/Github/Hanyang/model/nuri_4s/scene_nuri.xml'

try:
    model = mujoco.MjModel.from_xml_path(model_path)
except Exception as e:
    print(f"모델 로딩 오류: {e}")
    exit()
data = mujoco.MjData(model)

# --- 2. 키보드 콜백 함수 및 상태 변수 정의 ---
paused = False

def key_callback(keycode):
    if chr(keycode) == ' ':
        global paused
        paused = not paused

# --- 3. 뷰어 실행 ---
viewer = mujoco.viewer.launch_passive(model, data, key_callback=key_callback)

# --- 4. 제어기 및 경로 설정 ---
mass_matrix = np.zeros((model.nv, model.nv))
kp = np.array([100, 100, 100, 50, 50, 10])
kd = np.array([10, 10, 10, 5, 5, 2])

# 최종 목표 지점 설정
final_target_qpos = np.array([0, 0, 1.67, 0.0, 1.6, 0])
start_qpos = np.copy(data.qpos[:model.nu])
start_time = time.time()
duration = 5.0  # 목표까지 이동하는 데 걸리는 시간 (초)


# --- 5. 시뮬레이션 루프 ---
try:
    while viewer.is_running():
        step_start = time.time()

        # 1. 경과 시간 계산
        elapsed_time = time.time() - start_time
        if elapsed_time > duration:
            elapsed_time = duration # 목표 지점에 도달하면 멈춤

        # 2. 시간에 따른 중간 목표 위치와 속도 생성 (5차 다항식 사용으로 부드러운 움직임 구현)
        t = elapsed_time
        T = duration
        s = 10 * (t/T)**3 - 15 * (t/T)**4 + 6 * (t/T)**5
        s_dot = 30 * t**2 / T**3 - 60 * t**3 / T**4 + 30 * t**4 / T**5

        desired_qpos = start_qpos + s * (final_target_qpos - start_qpos)
        desired_qvel = s_dot * (final_target_qpos - start_qpos)

        # 3. 수정된 PD 제어: '중간 목표'를 추종하도록 변경
        pd_torque = kp * (desired_qpos - data.qpos[:model.nu]) + kd * (desired_qvel - data.qvel[:model.nu])


        # 동역학 계산 (Computed Torque)
        mujoco.mj_inverse(model, data)
        bias_torque = data.qfrc_bias
        mujoco.mj_fullM(model, mass_matrix, data.qM)
        gravity_torque = bias_torque + mass_matrix @ data.qacc
        # 최종 제어 토크
        if time.time() - start_time < duration:
            control_torque = gravity_torque + pd_torque
        else :
            control_torque = gravity_torque
        data.ctrl[:] = control_torque

        # 시뮬레이션 스텝 진행
        if not paused:
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