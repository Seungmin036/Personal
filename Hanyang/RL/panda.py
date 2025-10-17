import gymnasium as gym
import panda_gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
import torch # PyTorch가 설치되어 있는지 확인하기 위해 import
import time

# 4. 학습된 모델 불러오기
model_path = "/Users/shinseungmin/Documents/Github/ppo_panda_reach_model.zip"
model = PPO.load(model_path)


# 5. 학습된 모델로 성능 테스트
print("\n===== 학습된 모델 테스트 시작 =====")
# 테스트 시에는 render_mode="human"으로 실제 시뮬레이션 창을 띄웁니다.
eval_env = gym.make("PandaReach-v3", render_mode="human")
obs, info = eval_env.reset()

for i in range(10000):
    # model.predict()를 사용해 학습된 정책으로 최적의 행동을 결정합니다.
    action, _states = model.predict(obs, deterministic=True)
    
    obs, reward, terminated, truncated, info = eval_env.step(action)
    time.sleep(1/100)
    # 에피소드가 끝나면 환경을 리셋합니다.
    if terminated or truncated:
        print(f"성공! {i+1} 스텝 만에 목표 도달. 환경을 리셋합니다.")
        obs, info = eval_env.reset()

eval_env.close()