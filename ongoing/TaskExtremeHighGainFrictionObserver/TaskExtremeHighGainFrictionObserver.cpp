#include "TaskExtremeHighGainFrictionObserver.h"

template<typename ROBOT>
void TaskExtremeHighGainFrictionObserver<ROBOT>::initialize(ROBOT &robot, double delt)
{
    /**< Initialize the H-infinity controller with the given time step */
    robot.initHinfController(delt);
    /**< Reset the integral terms in the H-inifinity control algorithm */
    robot.resetHinfController();

    debug_cnt = 3;
    is_soft_estop_ = false;
    // file_.open("LPF_log.csv");

    _delT = delt;
    currentT_ = 0;
    reset(robot);
}

template<typename ROBOT>
void TaskExtremeHighGainFrictionObserver<ROBOT>::reset(ROBOT& robot)
{
    /**< Reset the integral terms in the H-inifinity control algorithm */
    robot.resetHinfController();
    
    /* Task-space controller reset */
    robot.computeFK(tpos_d_);
    /* Task-space controller reset end */

    /* Friction Observer initialization reset */
    fo_init_status_ = 0;
    fo_init_cnt_ = 0;
    K_lpf_vec_.setZero();
    theta_n_ = robot.q();
    dtheta_n_ = robot.qdot();
    tau_f_prev_ = Eigen::VectorXd::Zero(6);
    tau_j_prev_ = Eigen::VectorXd::Zero(6);

    Gamma_.resize(6, 6); Gamma_.setZero();
    Gamma_.diagonal() << 10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0;
    Gamma_p_.resize(6, 6); Gamma_p_.setZero();
    Gamma_p_.diagonal() << 100.0, 100.0, 100.0, 100.0, 100.0, 100.0;
    L_.resize(6, 6); L_.setZero();
    Lp_.resize(6, 6); Lp_.setZero();
    B_.resize(6, 6); B_.setZero();
    B_.diagonal() << 1.0, 1.0, 1.0, 0.8, 0.8, 0.8;

    /* Task-space controller initialization */
    robot.computeFK(tpos_d_);
    /* Task-space controller initialization end */

    /* JTS initialization init */
    // robotNominal_ = std::make_unique<ROBOT>(robot);
    jts_initialized_ = false;
    jts_init_cnt_ = 1;
    avg_cnt_ = static_cast<int>(2.0 / _delT);
    tau_j_bias_.setZero();
    /* JTS initialization init end */

    /* Friction Observer
     initialization init */
    fo_init_status_ = 0;
    K_lpf_vec_.setZero();
    fo_init_duration_ = 2.0;
    fo_init_cnt_ = 0;
    fo_init_cnt_total_ = static_cast<int>(fo_init_duration_ / _delT);
    /* Friction Observer initialization init end */

    std::cout << "========== Task RESET HAS CALLED ==========" << std::endl;
}

template<typename ROBOT>
void TaskExtremeHighGainFrictionObserver<ROBOT>::compute(ROBOT &robot, const LieGroup::Vector3D &gravDir,
                                               const MotionData &motionData, ControlData &controlData)
{
    robot.idyn_gravity(gravDir);
    JointVec gravity_compensation_torque = robot.tau();
    Kp_ = this->gain3;
    Kd_ = this->gain4;
    JointVec control_motor_torque; control_motor_torque.setZero();
    JointVec new_tau; new_tau.setZero();

    if (is_soft_estop_ || std::abs(robot.qdot().maxCoeff()) > 1.5) {
        controlData.controlTorque.tau = gravity_compensation_torque;
        is_soft_estop_ = true;
        std::cout << "EMERGENCY!!!" << std::endl;
        return;
    }
    
    // std::cout << "######################################################################################" << std::endl;
    // std::cout << "time: " << currentT_ << std::endl;

    /* JTS initialization */

    if (!jts_initialized_) {
        const double alpha = (jts_init_cnt_ - 1.0) / jts_init_cnt_;
        tau_j_ = controlData.controlTorque.tauJTS;
        tau_j_(1) = - controlData.controlTorque.tauJTS(1);

        JointVec tau_j_nominal;
        robot.idyn_gravity(gravDir);
        tau_j_nominal= robot.tau();
        tau_j_bias_ = alpha * tau_j_bias_ + (1 - alpha) * (tau_j_ - tau_j_nominal);
        jts_init_cnt_++;
        
        if (jts_init_cnt_ > avg_cnt_) {
            jts_initialized_ = true;
            jts_init_cnt_ = 1;
            std::cout<<"Joint torque sensor bias is initialized"<<std::endl;
            std::cout<<"bias : " << tau_j_bias_.transpose()<<std::endl;
        }

        control_motor_torque = - Kp_.cwiseProduct(robot.q() - motionData.motionPoint.qd) - Kd_.cwiseProduct( robot.qdot() - motionData.motionPoint.qdotd) + gravity_compensation_torque;
        new_tau = control_motor_torque;
    }
    /* JTS initialization end */
    else {
    // --------------------------------- TASK CONTORLLER -------------------------------
        // Kp_task_ = this->gain9;

        robot.computeFK(tpos_, tvel_);
        robot.computeJacobian(tpos_, tvel_, J_, Jdot_);

        ExtendedTaskJacobian JJtrans;
        JJtrans.setZero();
        JJtrans = J_*J_.transpose();
        Eigen::ColPivHouseholderQR<ExtendedTaskJacobian> qr(JJtrans);

        ExtendedTaskJacobian Jinv;
        Jinv.setZero();
        Jinv = J_.transpose()*qr.inverse();

        ExtendedTaskVec e, edot;
        e.setZero();
        edot.setZero();

        ExtendedPosition posDesired; posDesired.setZero();
        posDesired.R()  = tpos_d_.R();
        posDesired.r()  = tpos_d_.r();

        ExtendedVelocity velDesired; velDesired.setZero();

        robot.computeTaskErr(tpos_, tvel_, posDesired, velDesired, e, edot);

        JointVec tau_task;
        tau_task = Kp_task_.cwiseProduct(Jinv*e) - Kd_.cwiseProduct(robot.qdot()) + gravity_compensation_torque;
        std::cout << "tau_task: " << tau_task.transpose() << std::endl;
        JointVec friction_compensation_torque = friction_observer_L1_PD(robot, gravDir, tau_task, motionData, controlData);
        new_tau = tau_task - friction_compensation_torque;
    }
    
    currentT_ += _delT;
    controlData.controlTorque.tau = new_tau;
}

template<typename ROBOT>
typename TaskExtremeHighGainFrictionObserver<ROBOT>::JointVec
TaskExtremeHighGainFrictionObserver<ROBOT>::friction_observer_L1_PD(ROBOT &robot, const LieGroup::Vector3D &gravDir, const JointVec &Control_input,  const MotionData &motionData, ControlData &controlData)  
{
    Gamma_vec_ = this->gain5;
    Gamma_p_vec_ = this->gain6;
    B_vec_ = this->gain8;
    J_lpf_vec_ = this->gain0;
    
    if (Gamma_vec_.minCoeff() > 0)
        Gamma_.diagonal() << Gamma_vec_;
    if (Gamma_p_vec_.minCoeff() > 0)
        Gamma_p_.diagonal() << Gamma_p_vec_;
    if (B_vec_.minCoeff() > 0) 
        B_.diagonal() << B_vec_;
    
    if (this->gain7.minCoeff() > 0) {
        switch (fo_init_status_) {
            case 0:
                dK_lpf_vec_ = this->gain7 / (fo_init_duration_ / _delT);
                fo_init_cnt_ = 0;
                fo_init_status_ = 1;
                break;
            case 1:
                K_lpf_vec_ += dK_lpf_vec_;
                fo_init_cnt_++;
                if (fo_init_cnt_ > fo_init_cnt_total_) {
                    fo_init_status_ = 2;
                }
                break;
            case 2:
                K_lpf_vec_ = this->gain7;
                break;
            default:
                K_lpf_vec_.setZero();
        }
    }

    if (currentT_ - printT_ > 3.0) {
        std::cout << "K_lpf: " << K_lpf_vec_.transpose() << std::endl;
        std::cout << "Kp_task: " << Kp_task_.transpose() << std::endl;
        printT_ = currentT_;
    }
    
    // error calculate
    JointVec de_nr = dtheta_n_ - robot.qdot();
    JointVec e_nr = theta_n_ - robot.q();

    JointVec tau_f(6); JointVec sigma(6);

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
    sigma = - B_ * Gamma_ * (I + Gamma_* _delT).inverse() 
            * ((I + Gamma_p_ * _delT) * de_nr + Gamma_p_ * e_nr);
    Eigen::MatrixXd Alpha(6, 6); Alpha.setZero();
    Alpha.diagonal() = (- K_lpf_vec_ * _delT).array().exp();
    tau_f = Alpha * tau_f_prev_ + (I - Alpha) * sigma;

    // tau_j
    tau_j_ = controlData.controlTorque.tauJTS;
    tau_j_(1) = - controlData.controlTorque.tauJTS(1);
    tau_j_ -= tau_j_bias_;   // eliminate bias 

    // tau_J Low-Pass Filter
    Eigen::MatrixXd Beta(6, 6); Beta.setZero();
    Beta.diagonal() = (- J_lpf_vec_ * _delT).array().exp();
    tau_j_lpf_ = Beta * tau_j_prev_ + (I - Beta) * tau_j_;
    tau_j_prev_ = tau_j_lpf_;

    // state update
    JointVec ddtheta_n_ = B_.inverse() * ( Control_input - tau_j_lpf_ + sigma - tau_f );   
    dtheta_n_ += _delT * ddtheta_n_;
    theta_n_ += _delT * dtheta_n_;
    tau_f_prev_ = tau_f;

    return tau_f;
}
/**< Manifest to allow dynamic loading of the control algorithm */
POCO_BEGIN_MANIFEST(NRMKControl::ControlAlgorithmCreator)
        POCO_EXPORT_CLASS(TaskExtremeHighGainFrictionObserverCreator)
POCO_END_MANIFEST
