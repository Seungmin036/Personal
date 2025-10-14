
# 파일 이름: main_train_cartpole.py

import gymnasium as gym
from collections import deque
import numpy as np
import torch
import time

# ★★★ dqn_agent.py 파일에서 DQNAgent 클래스를 가져옵니다 ★★★
from dqn_agent import DQNAgent

# --- 환경 생성 ---
env = gym.make("LunarLander-v3")

# Define training parameters
num_episodes = 1000  # 학습이 잘 되도록 에피소드 수를 늘립니다.
max_steps_per_episode = 200 # CartPole-v1의 최대 스텝은 500입니다.
epsilon_start = 1.0
epsilon_end = 0.01 # 더 많이 탐험하도록 epsilon 최종값을 낮춥니다.
epsilon_decay_rate = 0.995 # 감쇠율을 약간 조정합니다.
lr = 5e-4 # 학습률 조정

# Initialize the DQNAgent
input_dim = env.observation_space.shape[0]
output_dim = env.action_space.n
agent = DQNAgent(state_size=input_dim, action_size=output_dim, seed=0, lr=lr)
scores = []
scores_window = deque(maxlen=100)
epsilon = epsilon_start

# Training loop
print("===== 학습 시작 =====")
for episode in range(1, num_episodes + 1):
    state, info = env.reset()
    score = 0
    
    # Run one episode
    for t in range(max_steps_per_episode):
        action = agent.act(state, epsilon)
        # ★ 수정된 부분: Gymnasium API에 맞게 step() 수정
        next_state, reward, terminated, truncated, info = env.step(action)
        done = terminated or truncated
        
        # ★ 수정된 부분: 훈련 루프는 agent.step()만 호출하도록 단순화
        agent.step(state, action, reward, next_state, done)
        
        state = next_state
        score += reward
        
        if done:
            break
            
    scores_window.append(score)
    scores.append(score)
    epsilon = max(epsilon_end, epsilon_decay_rate * epsilon)
    
    print(f'\rEpisode {episode}\tAverage Score: {np.mean(scores_window):.2f}', end="")
    if episode % 100 == 0:
        print(f'\rEpisode {episode}\tAverage Score: {np.mean(scores_window):.2f}')
    if np.mean(scores_window) >= 475.0: # CartPole-v1의 해결 기준은 100 에피소드 평균 475점 이상입니다.
        print(f'\nEnvironment solved in {episode-100:d} episodes!\tAverage Score: {np.mean(scores_window):.2f}')
        break

print("\n===== 학습 완료 =====")

# --- 학습된 모델 가중치 저장 ---
torch.save(agent.qnetwork_local.state_dict(), 'dqn_cartpole_weights.pth')
print("모델 가중치를 저장했습니다.")

# Visualize the agent's performance
print("\n===== 학습된 에이전트 시각화 =====")
env = gym.make("CLunarLander-v3", render_mode="human")
state, info = env.reset()
done = False
total_reward = 0

while not done:
    action = agent.act(state, eps=0.) # 평가 시에는 탐험(epsilon)을 하지 않습니다.
    next_state, reward, terminated, truncated, info = env.step(action)
    done = terminated or truncated
    state = next_state
    total_reward += reward
    time.sleep(1/60)

print(f"시각화 테스트 점수: {total_reward}")
env.close()