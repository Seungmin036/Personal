import mujoco
import mujoco.viewer
import numpy as np
import time

# --- 1. 모델 및 데이터 로드 ---
try:
    # 센서가 추가된 XML 파일을 로드해야 합니다.
    model = mujoco.MjModel.from_xml_path('/Users/shinseungmin/Documents/Github/Mujoco/model/franka_emika_panda/mjx_scene.xml')
except FileNotFoundError:
    print("오류: 'franka_emika_panda/panda.xml'을 찾을 수 없습니다.")
    exit()
except Exception as e:
    print(f"XML 파일 로딩 중 오류 발생: {e}")
    print("XML 파일에 <sensor> 섹션을 올바르게 추가했는지 확인하세요.")
    exit()

data = mujoco.MjData(model)

# --- 2. 뷰어 실행 및 시각화 옵션 설정 ---
viewer = mujoco.viewer.launch_passive(model, data)


# --- 4. 시뮬레이션 루프 ---
try:
    # 사인파 움직임을 위한 변수
    amplitude = 0.3
    frequency = 0.3

    while viewer.is_running():
        step_start = time.time()

        # --- 로봇 제어 (단순한 사인파 움직임) ---
        sim_time = data.time
        target_angle = amplitude * np.sin(2 * np.pi * frequency * sim_time)
        data.ctrl[0] = target_angle  # 1번 관절만 움직임
        data.ctrl[1] = target_angle * 0.5

        # --- 물리 스텝 진행 ---
        mujoco.mj_step(model, data)
        
        # --- 다양한 정보 수집 및 계산 ---
        # 1. 기본 정보
        qpos_joint1 = data.qpos[0]
        qvel_joint1 = data.qvel[0]
        
        # 2. 엔드 이펙터 정보 (순기구학)
        eef_site_id = model.site("gripper").id
        eef_pos = data.site(eef_site_id).xpos
        eef_mat = data.site(eef_site_id).xmat.reshape(3, 3)

        # 3. 센서 데이터
        eef_pos_sensor_id = model.sensor("eef_pos_sensor").id
        left_touch_id = model.sensor("left_finger_touch").id
        sensor_eef_pos = data.sensor(eef_pos_sensor_id).data
        sensor_touch_left = data.sensor(left_touch_id).data[0]

        # 4. 에너지 계산 (mj_energyPos, mj_energyVel 사용)
        mujoco.mj_energyPos(model, data)
        mujoco.mj_energyVel(model, data)
        potential_energy = data.energy[0]
        kinetic_energy = data.energy[1]
        
        # 5. 역동역학 (mj_inverse)
        # 현재 qpos, qvel, qacc 상태를 유지하기 위해 필요한 토크 계산
        mujoco.mj_inverse(model, data)
        required_torque = data.qfrc_inverse[:7].copy()

        # 6. 접촉 정보
        num_contacts = data.ncon
        contact_info = "None"
        if num_contacts > 0:
            # 첫 번째 접촉점의 정보를 예시로 표시
            contact = data.contact[0]
            geom1_name = model.geom(contact.geom1).name
            geom2_name = model.geom(contact.geom2).name
            contact_info = f"{geom1_name} and {geom2_name}"
        
        print(f"Time: {sim_time:.2f} s | Joint1 Pos: {qpos_joint1:.2f} rad | Joint1 Vel: {qvel_joint1:.2f} rad/s"
              f" | EEF Pos: {eef_pos} | Sensor EEF Pos: {sensor_eef_pos}"
              f" | Left Finger Touch: {sensor_touch_left:.2f}"
              f" | Kinetic Energy: {kinetic_energy:.2f} | Potential Energy: {potential_energy:.2f}"
              f" | Required Torque (Joint1): {required_torque[0]:.2f} Nm"
              f" | Contacts: {num_contacts} ({contact_info})")
        
        viewer.sync()

        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

except KeyboardInterrupt:
    print("시뮬레이션이 중지되었습니다.")
finally:
    viewer.close()
