import mujoco
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from stable_baselines3 import PPO

# '자세 유지'를 목표로 수정된 NuriEnv 클래스
class NuriStabilizeEnv(gym.Env):
    def __init__(self):
        super().__init__()
        self.model = mujoco.MjModel.from_xml_path("/Users/shinseungmin/Documents/Github/Mujoco/model/nuri_4s/nuri_4s.xml")
        self.data = mujoco.MjData(self.model)
        
        # 1. 관측 공간 수정: 목표 지점 정보가 필요 없으므로 qpos와 qvel만 사용
        obs_space_shape = (self.model.nq + self.model.nv,) # (6 + 6) = 12
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=obs_space_shape, dtype=np.float64)
        
        # 행동 공간은 그대로 유지
        self.action_space = spaces.Box(low=-10, high=10, shape=(self.model.nu,), dtype=np.float64)

    def _get_obs(self):
        # 2. 관측 내용 수정: qpos와 qvel 정보만 반환
        return np.concatenate([
            self.data.qpos,
            self.data.qvel,
        ]).astype(np.float64)

    def reset(self, seed=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        
        # 3. reset에서는 시뮬레이션 초기화만 수행
        return self._get_obs(), {}

    def step(self, action):
        # 4. 보상 함수는 '자세 유지' 목표이므로 그대로 사용
        target_qpos = np.zeros(self.model.nq)
        
        self.data.ctrl[:] = action
        mujoco.mj_step(self.model, self.data)

        position_error = np.sum(np.square(target_qpos - self.data.qpos))
        velocity_error = np.sum(np.square(self.data.qvel))
        control_cost = 0.01 * np.sum(np.square(action)) # 제어 비용 가중치 조절
        
        reward = -position_error - 0.1 * velocity_error - control_cost

        terminated = False
        return self._get_obs(), reward, terminated, False, {}


# 학습 시작
env = NuriStabilizeEnv() # 수정된 클래스 이름 사용
model = PPO("MlpPolicy", env, verbose=1, device="cpu") # device='cpu' 추가 권장

print("===== PPO 모델 학습을 시작합니다 (시간이 오래 걸릴 수 있습니다) =====")
model.learn(total_timesteps=1000000) # 의미있는 학습을 위해 타임스텝 증가
model.save("PPO_nuri_stabilize")

print("===== 모델 학습 완료 및 'PPO_nuri_stabilize.zip' 파일로 저장되었습니다. =====")
env.close()