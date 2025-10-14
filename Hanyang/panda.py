import mujoco
import mujoco.viewer
import numpy as np
import time
import os

# --- 1. 모델 및 데이터 로드 ---
# 사용자가 요청한 새로운 파일 경로로 수정
model_path = '/Users/shinseungmin/Documents/Github/Mujoco/model/franka_emika_panda/mjx_scene.xml'
try:
    model = mujoco.MjModel.from_xml_path(model_path)
except FileNotFoundError:
    print(f"오류: '{model_path}'을 찾을 수 없습니다.")
    exit()
except Exception as e:
    print(f"XML 파일 로딩 중 오류 발생: {e}")
    exit()

data = mujoco.MjData(model)

# --- 2. 키보드 콜백 함수 및 상태 변수 정의 ---
paused = False

def key_callback(keycode):
    """
    키보드 입력이 있을 때 호출되는 함수.
    스페이스바를 누르면 paused 상태를 토글합니다.
    """
    if chr(keycode) == ' ':
        global paused
        paused = not paused

# --- 3. 뷰어 실행 및 시각화 옵션 설정 ---
# key_callback 함수를 뷰어에 등록
viewer = mujoco.viewer.launch_passive(model, data, key_callback=key_callback)

viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CAMERA] = True


# --- 5. 시뮬레이션 루프 ---
try:
    amplitude = 0.3
    frequency = 0.3
    last_print_time = time.time()
    print_interval = 0.25

    while viewer.is_running():
        step_start = time.time()

        # paused 상태가 아닐 때만 시뮬레이션을 진행하고 정보를 업데이트
        if not paused:
            # --- 로봇 제어 ---
            sim_time = data.time
            # target_angle = amplitude * np.sin(2 * np.pi * frequency * sim_time)
            # data.ctrl[0] = target_angle
            # data.ctrl[1] = target_angle * 0.5

            # --- 물리 스텝 진행 ---
            mujoco.mj_step(model, data)
        
        # --- 일정 시간마다 터미널에 정보 출력 ---
        current_time = time.time()
        if current_time - last_print_time > print_interval:
            # --- 다양한 정보 수집 및 계산 ---
            qpos_joint1 = data.qpos[0]
            qvel_joint1 = data.qvel[0]
            
            # EE site 이름을 'gripper'로 수정
            eef_site_id = model.site("gripper").id
            eef_pos = data.site(eef_site_id).xpos
            
            # EE 위치 센서 이름을 'gripper_pos_sensor'로 가정하여 수정
            # 만약 mjx_scene.xml의 센서 이름이 다르다면 이 부분을 수정해야 합니다.
            gripper_pos_sensor_id = model.sensor("eef_pos_sensor").id
            sensor_eef_pos = data.sensor(gripper_pos_sensor_id).data

            # 참고: 터치 센서 이름은 알 수 없으므로 주석 처리합니다.
            # mjx_scene.xml 파일에서 실제 터치 센서 이름을 확인하고 주석을 해제하세요.
            left_touch_id = model.sensor("left_finger_touch").id
            sensor_touch_left = data.sensor(left_touch_id).data[0]
            right_touch_id = model.sensor("right_finger_touch").id
            sensor_touch_right = data.sensor(right_touch_id).data[0]

            mujoco.mj_energyPos(model, data)
            mujoco.mj_energyVel(model, data)
            potential_energy = data.energy[0]
            kinetic_energy = data.energy[1]
            mujoco.mj_inverse(model, data)
            required_torque = data.qfrc_inverse[:7].copy()
            num_contacts = data.ncon
            contact_info = "None"
            if num_contacts > 0:
                contact = data.contact[0]
                geom1_name = model.geom(contact.geom1).name
                geom2_name = model.geom(contact.geom2).name
                contact_info = f"{geom1_name} and {geom2_name}"
            
            
            
            print(f"시뮬레이션 시간: {sim_time:.3f} s")
            print(f"모델 자유도 (nv): {model.nv}")
            print("--- 키네마틱스 ---")
            print(f"관절 1 위치 (qpos): {qpos_joint1:.3f} rad")
            print(f"관절 1 속도 (qvel): {qvel_joint1:.3f} rad/s")
            print(f"EEF 위치 (xpos): {np.round(eef_pos, 2)}\n")
            print("--- 동역학 ---")
            print(f"위치 에너지: {potential_energy:.2f} J")
            print(f"운동 에너지: {kinetic_energy:.2f} J")
            print(f"관절 1 필요 토크(역동역학): {required_torque[0]:.2f} Nm\n")
            print("--- 센서 ---")
            print(f"EEF 위치 센서: {np.round(sensor_eef_pos, 2)}")
            print(f"왼쪽 손가락 터치: {sensor_touch_left:.3f} N (주석 처리됨)")

            last_print_time = current_time

        viewer.sync()

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0 and not paused:
            time.sleep(time_until_next_step)

except KeyboardInterrupt:
    print("시뮬레이션이 중지되었습니다.")
finally:
    if viewer.is_running():
        viewer.close()

