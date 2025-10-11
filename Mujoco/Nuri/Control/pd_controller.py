# pd_controller.py
import numpy as np
import mujoco
from mujoco import mjtTrn

# -------- nv -> nu 선택행렬(1-DOF joint actuator 가정) --------
def build_selector_S(model):
    S = np.zeros((model.nu, model.nv))
    for i in range(model.nu):
        if model.actuator_trntype[i] == mjtTrn.mjTRN_JOINT:
            jid = int(model.actuator_trnid[i] if getattr(model.actuator_trnid,'ndim',1)==1
                      else model.actuator_trnid[i,0])
            dof = model.jnt_dofadr[jid]  # 1-DOF 가정
            S[i, dof] = 1.0
    return S  # (nu, nv)

# -------------------- 중력보상 --------------------
class Gravity:
    def __init__(self, model, data):
        self.model = model
        self.data  = data

    def mass_matrix(self):
        mujoco.mj_forward(self.model, self.data)
        M = np.zeros((self.model.nv, self.model.nv))
        mujoco.mj_fullM(self.model, M, self.data.qM)
        return M

    def gravity_vector(self):
        v_save = self.data.qvel.copy()
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        G = self.data.qfrc_bias.copy()  # ≈ G(q)
        self.data.qvel[:] = v_save
        mujoco.mj_forward(self.model, self.data)  # 정합
        return G  # (nv,)

    def coriolis_vector(self):
        mujoco.mj_forward(self.model, self.data)
        CG = self.data.qfrc_bias.copy()  # C+G

        v_save = self.data.qvel.copy()
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        G = self.data.qfrc_bias.copy()

        self.data.qvel[:] = v_save
        mujoco.mj_forward(self.model, self.data)

        return CG - G  # (nv,)

    def tau_inverse_dynamics(self, qddot_des=None, split=True):
        """ tau = M @ qddot_des + C + G  (nv) """
        nv = self.model.nv
        if qddot_des is None:
            qddot_des = np.zeros(nv)
        else:
            qddot_des = np.asarray(qddot_des, float)
            if qddot_des.shape[0] != nv:
                raise ValueError("qddot_des length must equal nv")

        M = self.mass_matrix()
        if split:
            C = self.coriolis_vector()
            G = self.gravity_vector()
            tau = M @ qddot_des + C + G
        else:
            mujoco.mj_forward(self.model, self.data)
            CG = self.data.qfrc_bias.copy()
            tau = M @ qddot_des + CG
        return tau  # (nv,)

# -------------------- PD controller --------------------
class PDController:
    def __init__(self, model, data, Kp, Kd, target_final=None,
                 use_gravity=True, use_coriolis=False, clip_to_ctrlrange=True):
        self.model, self.data = model, data
        self.nu, self.nv = model.nu, model.nv
        self.S = build_selector_S(model)

        # 게인 (스칼라/벡터 모두 OK)
        Kp = np.asarray(Kp, float); Kd = np.asarray(Kd, float)
        if Kp.ndim == 0: Kp = np.full(self.nu, Kp)
        if Kd.ndim == 0: Kd = np.full(self.nu, Kd)
        self.Kp = np.diag(Kp); self.Kd = np.diag(Kd)

        # 목표각: 현재각으로 초기화 (actuator 순서)
        q_sel = self.S @ self.data.qpos
        self.q_des = q_sel.copy() if target_final is None else np.asarray(target_final, float)

        self.use_gravity  = use_gravity
        self.use_coriolis = use_coriolis
        self.clip         = clip_to_ctrlrange

        # ctrlrange
        cr = model.actuator_ctrlrange
        has = (cr[:,0] < cr[:,1])
        self.lo = np.where(has, cr[:,0], -np.inf)
        self.hi = np.where(has, cr[:,1],  np.inf)

        # 동역학 유틸
        self.dyn = Gravity(model, data)

    def step(self):
        # 상태 (actuator 순서)
        q  = self.S @ self.data.qpos  # (nu,)
        dq = self.S @ self.data.qvel  # (nu,)

        # PD
        u = (self.Kp @ (self.q_des - q)) - (self.Kd @ dq)  # (nu,)

        # 보상항 (nv -> nu 매핑)
        if self.use_gravity or self.use_coriolis:
            tau_nv = np.zeros(self.nv)
            if self.use_gravity and self.use_coriolis:
                tau_nv = self.dyn.tau_inverse_dynamics(qddot_des=np.zeros(self.nv), split=True)
            elif self.use_gravity:
                tau_nv = self.dyn.gravity_vector()
            elif self.use_coriolis:
                tau_nv = self.dyn.coriolis_vector()
            u += self.S @ tau_nv  # (nu,)

        if self.clip:
            u = np.clip(u, self.lo, self.hi)
        self.data.ctrl[:self.nu] = u