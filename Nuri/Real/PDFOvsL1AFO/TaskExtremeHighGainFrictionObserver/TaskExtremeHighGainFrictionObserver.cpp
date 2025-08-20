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
    file_.open("L1AFO.csv");

    _delT = delt;
    currentT_ = 0;
    printT_ = 0.0;
    /* JTS initialization init */
    // robotNominal_ = std::make_unique<ROBOT>(robot);
    jts_initialized_ = false;
    jts_init_cnt_ = 1;
    avg_cnt_ = static_cast<int>(2.0 / _delT);
    tau_j_bias_.setZero();
    traj_start_ = 10.0;
    traj_duration_ = 5.0;
    /* JTS initialization init end */
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
    Kp_task_.resize(6, 6); Kp_task_.setZero();
    Kd_task_.resize(6, 6); Kd_task_.setZero();
    
    /* Task-space controller initialization */
    robot.computeFK(tpos_d_);
    /* Task-space controller initialization end */

    /* Friction Observer initialization init */
    fo_init_status_ = 0;
    K_lpf_vec_.setZero();
    fo_init_duration_ = 2.0;
    fo_init_cnt_ = 0;
    fo_init_cnt_total_ = static_cast<int>(fo_init_duration_ / _delT);
    /* Friction Observer initialization init end */

    /* Friction Observer initialization reset end */
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
    PW_ = this->gain9;
    JointVec control_motor_torque; control_motor_torque.setZero();
    JointVec new_tau; new_tau.setZero();

    if (is_soft_estop_ || std::abs(robot.qdot().maxCoeff()) > 1.3 || new_tau.norm() > 200.0 ) {
        controlData.controlTorque.tau = gravity_compensation_torque;
        is_soft_estop_ = true;
        std::cout << "EMERGENCY!!" << std::endl;
        return;
    }
    const double pw_target_ = 6.29;
    const double pw_tol_ = 1e-6;
    if  (std::abs(PW_[0] - pw_target_) > pw_tol_) {  
        controlData.controlTorque.tau = gravity_compensation_torque;
        if (std::abs(PW_[0]) > pw_tol_)
            std::cout << "PW error" << std::endl;
        return;
    }

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
            std::cout<<"bias_task : " << tau_j_bias_.transpose()<<std::endl;
        }

        control_motor_torque = gravity_compensation_torque;
        new_tau = control_motor_torque;
    }
    /* JTS initialization end */
    else {
    // // --------------------------------- TASK CONTORLLER -------------------------------
        Kp_task_vec_ = this->gain1;
        Kd_task_vec_ = this->gain2;
        if (Kp_task_vec_.minCoeff() > 0)
            Kp_task_.diagonal() << Kp_task_vec_;
        if (Kd_task_vec_.minCoeff() > 0)
            Kd_task_.diagonal() << Kd_task_vec_;

        robot.computeFK(tpos_, tvel_);
        robot.computeJacobian(tpos_, tvel_, J_, Jdot_);
//-------------------------------------------------------------
        // 하드 코딩
        // Eigen::Vector3d r_;
        // r_ << 0, 0, 0.2 ;    
        
        // Eigen::Vector3d r_new = tpos.r() + tpos.R() * r_;
        // Eigen::Matrix3d R_new = tpos.R();
        // ExtendedPosition tpos_new;
        // tpos_new.r() = r_new;
        // tpos_new.R() = R_new;

        // Eigen::Vector3d v_new = tvel.tail<3>() + tvel.head<3>().cross(r_);
        // Eigen::Vector3d w_new = tvel.head<3>();
        // Eigen::Matrix<double,6,1> tvel_new; 
        // tvel_new << w_new, v_new;

        // robot.computeFK(tpos_new, tvel_new);
        // robot.computeJacobian(tpos_new, tvel_new, J_, Jdot_);

        // API 함수
        // LieGroup::Rotation R_tcp;
        // LieGroup::Displacement r_tcp;

        // R_tcp.setIdentity();           
        // r_tcp << 0, 0, 0.18;           
        // robot.setTtarget(R_tcp, r_tcp);

        // robot.computeFK(tpos_, tvel_);
        // robot.computeJacobian(tpos_, tvel_, J_, Jdot_);

        ExtendedTaskVec e, edot;
        e.setZero();
        edot.setZero();
        
        ExtendedPosition posDesired;
        posDesired.R()  = motionData.motionPoint.pd.R();
        posDesired.r()  = motionData.motionPoint.pd.r();
        Eigen::Vector3d pos_desired_linear; 
        Eigen::Vector3d vel_desired_linear;
        Eigen::Vector3d p0, pf, v0, vf, a0, af;
        v0.setZero(); a0.setZero();
        vf.setZero(); af.setZero();

        if (!traj_initialized_ && currentT_ >= traj_start_) {
            traj_initialized_ = true;
            startPos_ = tpos_.r();
            pos_desired_linear = startPos_;
            std::cout<<"startpos"<<startPos_ << std::endl;
            p0 = startPos_;
            vel_desired_linear.setZero();
        }
        else if (traj_initialized_ && currentT_ >= traj_start_ && currentT_ < traj_start_ + traj_duration_) {
            p0 = startPos_;
            pf = startPos_ + Eigen::Vector3d(0, -0.2, 0);
            pos_desired_linear = computeQuinticSpline(p0, pf, v0, vf, a0, af, currentT_ - traj_start_, traj_duration_);
            vel_desired_linear = computeQuinticSplineVelocity(p0, pf, v0, vf, a0, af, currentT_ - traj_start_, traj_duration_);
        }
        else if (traj_initialized_ && currentT_ >= traj_start_ + traj_duration_ && currentT_ < traj_start_ + 2 * traj_duration_) {
            p0 = startPos_ + Eigen::Vector3d(0, -0.2, 0);
            pf = p0 + Eigen::Vector3d(0, 0, -0.2);
            pos_desired_linear = computeQuinticSpline(p0, pf, v0, vf, a0, af, currentT_ - (traj_start_ + traj_duration_), traj_duration_);
            vel_desired_linear = computeQuinticSplineVelocity(p0, pf, v0, vf, a0, af, currentT_ - (traj_start_ + traj_duration_), traj_duration_);
        }
        else if (traj_initialized_ && currentT_ >= traj_start_ + 2 * traj_duration_ && currentT_ < traj_start_ + 3 * traj_duration_) {
            p0 = startPos_ + Eigen::Vector3d(0, -0.2, -0.2);
            pf = p0 + Eigen::Vector3d(0, 0.2, 0);
            pos_desired_linear = computeQuinticSpline(p0, pf, v0, vf, a0, af, currentT_ - (traj_start_ + 2 * traj_duration_), traj_duration_);
            vel_desired_linear = computeQuinticSplineVelocity(p0, pf, v0, vf, a0, af, currentT_ - (traj_start_ + 2 * traj_duration_), traj_duration_);
        }
        else if (traj_initialized_ && currentT_ >= traj_start_ + 3 * traj_duration_ && currentT_ < traj_start_ + 4 * traj_duration_) {
            p0 = startPos_ + Eigen::Vector3d(0, 0, -0.2);
            pf = p0 + Eigen::Vector3d(0, 0, 0.2);
            pos_desired_linear = computeQuinticSpline(p0, pf, v0, vf, a0, af, currentT_ - (traj_start_ + 3 * traj_duration_), traj_duration_);
            vel_desired_linear = computeQuinticSplineVelocity(p0, pf, v0, vf, a0, af, currentT_ - (traj_start_ + 3 * traj_duration_), traj_duration_);
        }
        else if (traj_initialized_ && currentT_ >= traj_start_ + 4 * traj_duration_){
            pos_desired_linear = startPos_;
            vel_desired_linear.setZero();
        }
        else{
            pos_desired_linear = motionData.motionPoint.pd.r();
            vel_desired_linear.setZero();
        }

        posDesired.r() = pos_desired_linear; 
        std::cout << posDesired.r().transpose() << std::endl;
        ExtendedVelocity velDesired; 
        velDesired.v()  = vel_desired_linear;   
        velDesired.w().setZero();

        robot.computeTaskErr(tpos_, tvel_, posDesired, velDesired, e, edot);

        JointVec tau_task; 
        tau_task = J_.transpose() * ( Kp_task_ * e + Kd_task_ * edot) + gravity_compensation_torque; 
        JointVec friction_compensation_torque = friction_observer_L1_PD(robot, gravDir, tau_task, motionData, controlData);
        new_tau = tau_task - friction_compensation_torque; 
        
        controlData.controlReserved.qdotres2 = friction_compensation_torque;
        Eigen::Vector3d toolpos; toolpos = tpos_.r();
        Eigen::Vector3d desired_toolpos; desired_toolpos = posDesired.r();
        Eigen::Vector3d desired_toolvel; desired_toolvel = velDesired.v(); 
        JointVec error_; error_ = e;
        file_ << currentT_ <<",";
        for (int i = 0; i < 6; i++) {   
            file_ << error_(i) << ",";
        }
        for (int i = 0; i < 3; i++) {   
            file_ << desired_toolpos(i) << ",";
        }
        for (int i = 0; i < 3; i++) {   
            file_ << toolpos(i) << ",";
        }
        for (int i = 0; i < 3; i++) {   
            file_ << vel_desired_linear(i) << ",";
        }

        file_ << std::endl;
    }
    currentT_ += _delT;
    controlData.controlTorque.tau = new_tau;
    // controlData.controlTorque.tau = gravity_compensation_torque;
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
template<typename ROBOT>
Eigen::Vector3d TaskExtremeHighGainFrictionObserver<ROBOT>::computeQuinticSpline(const Eigen::Vector3d& p0, const Eigen::Vector3d& pf,
                                     const Eigen::Vector3d& v0, const Eigen::Vector3d& vf,
                                     const Eigen::Vector3d& a0, const Eigen::Vector3d& af,
                                     double t, double T)
{
    Eigen::Vector3d pos;
    double term = t / T;
    double term2 = term * term;
    double term3 = term2 * term;
    double term4 = term3 * term;
    double term5 = term4 * term;

    Eigen::VectorXd coeffs(6);
    coeffs << 1, term, term2, term3, term4, term5;

    for (int i = 0; i < 3; ++i) {
        Eigen::VectorXd b(6);
        b << p0(i), v0(i)*T, a0(i)*T*T/2.0,
             10*(pf(i)-p0(i)) - 6*v0(i)*T - 1.5*a0(i)*T*T - 4*vf(i)*T + 0.5*af(i)*T*T,
            -15*(pf(i)-p0(i)) + 8*v0(i)*T + 1.5*a0(i)*T*T + 7*vf(i)*T - af(i)*T*T,
              6*(pf(i)-p0(i)) - 3*v0(i)*T - 0.5*a0(i)*T*T - 3*vf(i)*T + 0.5*af(i)*T*T;
        pos(i) = coeffs.dot(b);
    }

    return pos;
}
template<typename ROBOT>
Eigen::Vector3d TaskExtremeHighGainFrictionObserver<ROBOT>::computeQuinticSplineVelocity(const Eigen::Vector3d& p0, const Eigen::Vector3d& pf,
                                             const Eigen::Vector3d& v0, const Eigen::Vector3d& vf,
                                             const Eigen::Vector3d& a0, const Eigen::Vector3d& af,
                                             double t, double T)
{
    Eigen::Vector3d vel;
    double term = t / T;
    double term2 = term * term;
    double term3 = term2 * term;
    double term4 = term3 * term;

    Eigen::VectorXd dcoeffs(6);
    dcoeffs << 0,
               1.0 / T,
               2.0 * term / T,
               3.0 * term2 / T,
               4.0 * term3 / T,
               5.0 * term4 / T;

    for (int i = 0; i < 3; ++i) {
        Eigen::VectorXd b(6);
        b << p0(i), v0(i)*T, a0(i)*T*T/2.0,
             10*(pf(i)-p0(i)) - 6*v0(i)*T - 1.5*a0(i)*T*T - 4*vf(i)*T + 0.5*af(i)*T*T,
            -15*(pf(i)-p0(i)) + 8*v0(i)*T + 1.5*a0(i)*T*T + 7*vf(i)*T - af(i)*T*T,
              6*(pf(i)-p0(i)) - 3*v0(i)*T - 0.5*a0(i)*T*T - 3*vf(i)*T + 0.5*af(i)*T*T;

        vel(i) = dcoeffs.dot(b);
    }

    return vel;
}

/**< Manifest to allow dynamic loading of the control algorithm */
POCO_BEGIN_MANIFEST(NRMKControl::ControlAlgorithmCreator)
        POCO_EXPORT_CLASS(TaskExtremeHighGainFrictionObserverCreator)
POCO_END_MANIFEST
