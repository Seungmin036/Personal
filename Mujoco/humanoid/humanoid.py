import mujoco
import mujoco.viewer
import numpy as np
from stable_baselines3 import PPO

# XML 경로
XML = "/Users/shinseungmin/Documents/Github/Mujoco/model/humanoid/humanoid.xml"

# 모델과 데이터 로드
model_mj = mujoco.MjModel.from_xml_path(XML)
data_mj = mujoco.MjData(model_mj)

# --- 1. 학습된 PPO 모델 불러오기 ---
# 'ppo_humanoid.zip' 파일은 아래 2번 과정(학습)을 통해 생성됩니다.
try:
    agent = PPO.load("ppo_humanoid.zip")
    print("학습된 모델을 성공적으로 불러왔습니다.")
except FileNotFoundError:
    print("오류: 학습된 모델 파일('ppo_humanoid.zip')을 찾을 수 없습니다.")
    print("아래 2번 학습 코드를 먼저 실행하여 모델을 생성해주세요.")
    exit()

# 상태(State) 정의 함수
def get_state(data):
    return np.concatenate([data.qpos, data.qvel])

# 뷰어 실행
with mujoco.viewer.launch_passive(model_mj, data_mj) as viewer:
    # 시뮬레이션 리셋
    mujoco.mj_resetData(model_mj, data_mj)
    
    while viewer.is_running():
        # --- 2. '뇌'의 역할이 PPO 모델로 대체됨 ---
        # 현재 상태를 관찰
        state = get_state(data_mj)
        
        # PPO 모델이 상태를 기반으로 최적의 행동을 예측
        # deterministic=True는 가장 확률이 높은 행동 하나만을 선택하겠다는 의미
        action, _states = agent.predict(state, deterministic=True)
        
        # 예측된 행동을 시뮬레이션에 적용
        data_mj.ctrl[:] = action
        
        mujoco.mj_step(model_mj, data_mj)
        viewer.sync()