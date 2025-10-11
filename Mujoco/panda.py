
import mujoco
import mujoco.viewer
import time
import numpy as np

class PandaController:
    
    def __init__(self):
        try:
            self.model = mujoco.MjModel.from_xml_path('/Users/shinseungmin/Documents/Github/Mujoco/model/franka_emika_panda/mjx_scene.xml')
        except FileNotFoundError:
            print("오류: xml 파일을 찾을 수 없습니다.")
            exit()
            
        self.data = mujoco.MjData(self.model)
        # 주요 ID 찾기
        self.site_name = "gripper"
        self.site_id = self.model.site(self.site_name).id
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        
        self.data.ctrl[:7] = np.zeros(7)
        tiem_step = 0.01
        
    def move_to_joint_angles(self, target_qpos, duration=3.0):
        
        initial_qpos = self.data.qpos[:7].copy()
        start_time = self.data.time
        
        while (self.data.time - start_time) < duration:
            step_start = time.time()
        
            progress = (self.data.time - start_time) / duration
            interpolated_qpos = initial_qpos + progress * (target_qpos - initial_qpos)
            
            # --- 2. PD + 중력 보상 제어기로 토크 계산 ---
            current_qpos = self.data.qpos[:7]
            current_qvel = self.data.qvel[:7]
            
            position_error = interpolated_qpos - current_qpos
            velocity_error = -current_qvel
            current_ee_pos = self.data.site(self.site_id).xpos

            
            jacp = np.zeros((3, self.model.nv))
            jacr = np.zeros((3, self.model.nv))
            mujoco.mj_jac(self.model, self.data, jacp, jacr, current_ee_pos, self.site_id)
            print(jacp)
            # PD 제어 토크 계산
            pd_torque = self.kp * position_error + self.kv * velocity_error
      
      
            input_torque = pd_torque + self.data.qfrc_bias[:7]
            
            self.data.ctrl[:7] = input_torque
            
            mujoco.mj_step(self.model, self.data)
            self.viewer.sync()

            time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
        
        # 이동이 끝난 후 목표 자세를 유지하도록 설정
        self.data.ctrl[:7] = target_qpos
        print("이동 완료.")


if __name__ == '__main__':
    # 이 파일을 직접 실행할 경우 테스트 코드를 실행합니다.
    robot = PandaController()
    
    # 테스트: 목표 관절 자세 정의 (7개 관절, 라디안)
    home_position = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785])
    target_position = np.array([0.5, 0.2, 0.0, -1.0, 0.0, 1.2, 0.0])
    while robot.viewer.is_running():
            # home 자세로 3초간 이동
        robot.move_to_joint_angles(home_position, duration=3.0)
        # target 자세로 3초간 이동
        robot.move_to_joint_angles(target_position, duration=3.0)
        
    robot.close()

