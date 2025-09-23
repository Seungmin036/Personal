import mujoco
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env

# 1. MuJoCo 시뮬레이션을 Gym 환경으로 만들기
class HumanoidEnv(gym.Env):
    def __init__(self):
        super().__init__()
        self.model = mujoco.MjModel.from_xml_path("/Users/shinseungmin/Documents/Github/Mujoco/model/nuri_4s/nuri_4s.xml")
        self.data = mujoco.MjData(self.model)
        
        # 상태와 행동의 범위를 정의 (라이브러리가 요구하는 형식)
        obs_space_shape = (self.model.nq + self.model.nv,)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=obs_space_shape, dtype=np.float64)
        
        # Action Space: ctrl (21)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(self.model.nu,), dtype=np.float64)

    def _get_obs(self):
        return np.concatenate([self.data.qpos, self.data.qvel]).astype(np.float64)

    def reset(self, seed=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        return self._get_obs(), {}

    def step(self, action):
        # 행동 적용 및 시뮬레이션 진행
        self.data.ctrl[:] = action
        mujoco.mj_step(self.model, self.data)

        # 보상 계산
        forward_reward = self.data.qvel[0]
        control_cost = .5 * np.square(action).sum()
        alive_bonus = 5.0
        reward = forward_reward - control_cost + alive_bonus
        
        # 종료 조건 확인 (넘어졌는지)
        terminated = not (1.0 < self.data.qpos[2] < 2.0)
        
        return self._get_obs(), reward, terminated, False, {}

# 2. 환경 및 PPO 모델 생성
env = HumanoidEnv()

# 3. 모델 학습 시작
# MlpPolicy: 다층 퍼셉트론(기본 신경망) 정책을 사용
# verbose=1: 학습 진행 상황을 출력
model = PPO("MlpPolicy", env, verbose=1, device="mps")

print("===== PPO 모델 학습을 시작합니다 (시간이 오래 걸릴 수 있습니다) =====")
# total_timesteps: 총 학습할 스텝 수. 숫자가 클수록 더 오래, 더 많이 학습합니다.
model.learn(total_timesteps=1000000)

# 4. 학습된 모델 저장
model.save("ppo_humanoid")
print("===== 모델 학습 완료 및 'ppo_humanoid.zip' 파일로 저장되었습니다. =====")

env.close()