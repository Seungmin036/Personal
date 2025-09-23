import mujoco
import mujoco.viewer
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from stable_baselines3 import PPO

# 1. 학습할 때 사용했던 환경 클래스를 그대로 가져와야 합니다.
class NuriStabilizeEnv(gym.Env):
    def __init__(self):
        super().__init__()
        # XML 파일 경로는 실제 위치에 맞게 확인해주세요.
        self.model = mujoco.MjModel.from_xml_path("/Users/shinseungmin/Documents/Github/Mujoco/model/nuri_4s/nuri_4s.xml")
        self.data = mujoco.MjData(self.model)
        
        obs_space_shape = (self.model.nq + self.model.nv,)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=obs_space_shape, dtype=np.float64)
        self.action_space = spaces.Box(low=-10, high=10, shape=(self.model.nu,), dtype=np.float64)

    def _get_obs(self):
        return np.concatenate([self.data.qpos, self.data.qvel]).astype(np.float64)

    def reset(self, seed=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        return self._get_obs(), {}

    def step(self, action):
        target_qpos = np.zeros(self.model.nq)
        self.data.ctrl[:] = action
        mujoco.mj_step(self.model, self.data)

        position_error = np.sum(np.square(target_qpos - self.data.qpos))
        velocity_error = np.sum(np.square(self.data.qvel))
        control_cost = 0.01 * np.sum(np.square(action))
        
        reward = -position_error - 0.1 * velocity_error - control_cost

        terminated = False
        return self._get_obs(), reward, terminated, False, {}

# --- 여기서부터 테스트 로직 시작 ---

# 2. 저장된 PPO 모델 불러오기 (파일 이름 확인!)
# 학습 시 저장한 파일 이름이 "PPO_nuri_stabilize.zip" 이라면 아래와 같이 수정하세요.
model = PPO.load("PPO_nuri_stabilize") # <--- 여기에 생성된 .zip 파일 이름을 넣으세요.

# 3. 테스트를 위한 환경 생성
env = NuriStabilizeEnv()
obs, _ = env.reset()

# 4. 뷰어를 통해 시뮬레이션 실행
print("학습된 에이전트의 움직임을 확인합니다. 뷰어 창을 닫으면 종료됩니다.")
with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
    while viewer.is_running():
        # 5. 에이전트가 현재 상태(obs)를 보고 행동(action)을 결정
        # deterministic=True는 학습된 대로 가장 확률 높은 행동만 하도록 설정
        action, _states = model.predict(obs, deterministic=True)
        
        # 6. 결정된 행동을 환경에 적용
        obs, reward, terminated, truncated, info = env.step(action)
        
        # (옵션) 에피소드가 끝나면 리셋 (자세 유지 과제에서는 거의 발생 안 함)
        if terminated or truncated:
            obs, _ = env.reset()

        # 7. 뷰어 화면 업데이트
        viewer.sync()

env.close()
print("테스트가 종료되었습니다.")