import pybullet as p
import pybullet_data
import time

# 1. 서버 연결 및 환경 설정
physicsClient = p.connect(p.GUI) 
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# 2. 모든 모델 불러오기 (로봇 포함)
# 바닥과 테이블
planeId = p.loadURDF("plane.urdf")
tableId = p.loadURDF("table/table.urdf", basePosition=[0, 0, 0])
# ⭐ 로봇팔 (시뮬레이션 루프 이전으로 이동)
robot_start_pos = [0, 0, 0.63]
robot_start_orn = p.getQuaternionFromEuler([0, 0, 0])
pandaId = p.loadURDF("franka_panda/panda.urdf", robot_start_pos, robot_start_orn, useFixedBase=True)

# 커스텀 병 모델
p.setAdditionalSearchPath("./my_models") 
bottle_startpos = [0.5, 0, 0.8]
bottleId = p.loadURDF("bottle.urdf", bottle_startpos)


# 3. 시뮬레이션 루프 실행
try:
    for i in range(100000):
        # 이 안에서 로봇을 제어하거나 상태를 관찰합니다.
        p.stepSimulation()
finally:
    # 4. 모든 작업이 끝나면 연결 종료
    p.disconnect()