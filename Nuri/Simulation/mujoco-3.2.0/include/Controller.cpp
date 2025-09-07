#include "Controller.hpp"

Controller::Controller(pinocchio::Model pinocchio_model)
:model_(pinocchio_model)
{
    data_ = pinocchio::Data(model_);
    // for(int i=0;i<model_.frames.size();i++)
    // {
    //     std::cout<< model_.frames[i] <<std::endl;
    // }

    Kp_.resize(6,6); Kp_.setZero();
    Kp_.diagonal() << 500, 500, 500, 300, 100, 30;
    Kd_.resize(6,6); Kd_.setZero();
    Kd_.diagonal() << 5, 5, 5, 2, 2, 1;

    Kp_task_.resize(6,6); Kp_task_.setZero();
    Kp_task_.diagonal() << 0, 0, 0, 2500, 2500, 2500;
    Kd_task_.resize(6,6); Kd_task_.setZero();
    Kd_task_.diagonal() << 0, 0, 0, 300, 300, 300;ㄴ
}

Eigen::VectorXd Controller::computeGravityCompensation(const mjModel* m, mjData* d)
{
    Eigen::VectorXd q(6);
    Eigen::VectorXd qdot(6);
    q << d->qpos[0], d->qpos[1], d->qpos[2], d->qpos[3], d->qpos[4], d->qpos[5];
    qdot << d->qvel[0], d->qvel[1], d->qvel[2], d->qvel[3], d->qvel[4], d->qvel[5];
    return pinocchio::computeGeneralizedGravity(model_,data_,q);
}

Eigen::VectorXd Controller::computeDotFixedPD(const mjModel* m, mjData* d, Eigen::Vector3d& fixed_pt)
{
    Eigen::VectorXd q(6);
    Eigen::VectorXd qdot(6);
    q << d->qpos[0], d->qpos[1], d->qpos[2], d->qpos[3], d->qpos[4], d->qpos[5];
    qdot << d->qvel[0], d->qvel[1], d->qvel[2], d->qvel[3], d->qvel[4], d->qvel[5];

    pinocchio::FrameIndex link6_id = model_.getFrameId("nuri_4s_link6");

    pinocchio::forwardKinematics(model_,data_,q,qdot);
    pinocchio::updateFramePlacements(model_,data_);

    Eigen::Vector3d link6_pos = data_.oMf[link6_id].translation();
    Eigen::Vector3d link6_lin_vel = pinocchio::getFrameVelocity(model_,data_,link6_id,pinocchio::LOCAL_WORLD_ALIGNED).linear();
    
    Eigen::MatrixXd link6_Jacobian; link6_Jacobian.resize(6,6);
    pinocchio::computeFrameJacobian(model_, data_, q, link6_id,pinocchio::LOCAL_WORLD_ALIGNED, link6_Jacobian);

    Eigen::Vector3d e = fixed_pt - link6_pos;
    Eigen::Vector3d edot = -link6_lin_vel;

    Eigen::MatrixXd J_rcm = link6_Jacobian.topRows(3);
    Eigen::VectorXd tau_rcm = J_rcm.transpose()*(Kp_task_.bottomRightCorner<3,3>()*e + Kd_task_.bottomRightCorner<3,3>()*edot) + computeGravityCompensation(m,d);

    Eigen::MatrixXd JJtrans = J_rcm * J_rcm.transpose();
    Eigen::MatrixXd J_rcm_inv = J_rcm.transpose()*JJtrans.inverse();

    Eigen::MatrixXd N = Eigen::MatrixXd::Identity(6, 6) - (J_rcm_inv * J_rcm).transpose();
    tau_rcm += N * Kd_.cwiseProduct(-qdot);

    return tau_rcm;
}

Eigen::VectorXd Controller::computeLineFixedPD(const mjModel* m, mjData* d, Eigen::Vector3d& fixed_pt, Eigen::Vector3d& fixed_vec)
{
    Eigen::VectorXd q(6);
    Eigen::VectorXd qdot(6);
    q << d->qpos[0], d->qpos[1], d->qpos[2], d->qpos[3], d->qpos[4], d->qpos[5];
    qdot << d->qvel[0], d->qvel[1], d->qvel[2], d->qvel[3], d->qvel[4], d->qvel[5];

    pinocchio::FrameIndex link6_id = model_.getFrameId("nuri_4s_link6");

    pinocchio::forwardKinematics(model_,data_,q,qdot);
    pinocchio::updateFramePlacements(model_,data_);

    Eigen::Vector3d link6_p = data_.oMf[link6_id].translation();
    Eigen::Matrix3d link6_R = data_.oMf[link6_id].rotation();
    Eigen::Vector3d link6_Zaxis = link6_R.col(2);
    Eigen::Vector3d link6_v = pinocchio::getFrameVelocity(model_,data_,link6_id,pinocchio::LOCAL_WORLD_ALIGNED).linear();
    Eigen::Vector3d link6_w = pinocchio::getFrameVelocity(model_,data_,link6_id,pinocchio::LOCAL_WORLD_ALIGNED).angular();

    Eigen::MatrixXd J; J.resize(6,6);
    pinocchio::computeFrameJacobian(model_, data_, q, link6_id, pinocchio::LOCAL_WORLD_ALIGNED, J);
    Eigen::MatrixXd lin_J = J.topRows(3);
    Eigen::MatrixXd ang_J = J.bottomRows(3);

    Eigen::Vector3d e3 = link6_Zaxis.cross(fixed_vec);
    Eigen::Vector3d w_perp = link6_w - link6_w.dot(link6_Zaxis)*link6_Zaxis;

    Eigen::Vector3d a = fixed_vec.unitOrthogonal();
    Eigen::Vector3d b = fixed_vec.cross(a);
    Eigen::Matrix<double,2,3> P_orient; P_orient << a.transpose(), b.transpose();
    Eigen::Vector2d e_dir = P_orient * e3;
    Eigen::Vector2d w_perp2 = P_orient * w_perp;

    Eigen::MatrixXd Jw2 = P_orient * lin_J;
    Eigen::Vector2d m_dir2 = 2500 * e_dir - 300 * w_perp2;
    Eigen::VectorXd tau_dir = Jw2.transpose() * m_dir2;

    Eigen::Vector3d r = link6_p - fixed_pt;
    Eigen::Matrix3d P = Eigen::Matrix3d::Identity() - link6_Zaxis * link6_Zaxis.transpose();
    Eigen::Vector3d e_rcm3 = P * r;
    Eigen::Vector3d edot_rcm3 = P * link6_v;

    // 2DOF로 압축 (z에 직교하는 기저로 투영)
    Eigen::Vector3d u1 = link6_Zaxis.unitOrthogonal();
    Eigen::Vector3d u2 = link6_Zaxis.cross(u1);
    Eigen::Matrix<double,2,3> P_pos; P_pos << u1.transpose(), u2.transpose();

    Eigen::Vector2d e_rcm = P_pos * e_rcm3;
    Eigen::Vector2d edot_rcm = P_pos * edot_rcm3;

    // 투영 야코비안 (선속도 부분)
    Eigen::MatrixXd Jv2 = P_pos * ang_J;

    Eigen::Vector2d f_rcm2 = 2500 * e_rcm - 300 * edot_rcm;
    Eigen::VectorXd tau_rcm = Jv2.transpose() * f_rcm2;

    tau_rcm += (tau_dir + computeGravityCompensation(m,d));

    Eigen::MatrixXd J_rcm(4, 6);
    J_rcm.topRows<2>()    = Jv2;   // RCM 2DOF (선속도 쪽)
    J_rcm.bottomRows<2>() = Jw2;   // 방향 2DOF (각속도 쪽)
    
    Eigen::MatrixXd JJtrans = J_rcm * J_rcm.transpose();
    Eigen::MatrixXd J_rcm_inv = J_rcm.transpose()*JJtrans.inverse();

    Eigen::MatrixXd N = Eigen::MatrixXd::Identity(6, 6) - (J_rcm_inv * J_rcm).transpose();
    tau_rcm += N * Kd_.cwiseProduct(-qdot);

    return tau_rcm;

}