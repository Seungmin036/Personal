#include "TaskPDFrictionObserver.h"

template<typename ROBOT>
void TaskPDFrictionObserver<ROBOT>::initialize(ROBOT &robot, double delt)
{
    _robotNominal = std::make_unique<ROBOT>(robot);
    reset(robot);
    debug_cnt = 3;
    is_soft_estop_ = false;
    file_.open("PDFO.csv");
    _delT = delt;
    currentT_ = 0;
    printT_ = 0.0;
      /**< initialization */
    traj_start_ = 10.0;
    traj_duration_ = 5.0;
    traj_ << 0.1, 0.1, 0.1;
    J_.setZero(); Jdot_.setZero();

    // temp
    save_integral_.resize(6);
    save_integral_.setZero();
}

template<typename ROBOT>
void TaskPDFrictionObserver<ROBOT>::reset(ROBOT& robot)
{ 
    /* Task-space controller reset end */
    theta_n_ = robot.q();
    dtheta_n_ = robot.qdot();

    fo_init_status_ = 0;
    fo_init_cnt_ = 0;
    L_vec_.setZero();

    L_.resize(6, 6); L_.setZero();
    L_.diagonal() << 80.0, 80.0, 80.0, 80.0, 80.0, 80.0;
    Lp_.resize(6, 6); Lp_.setZero();
    Lp_.diagonal() << 40.0, 40.0, 40.0, 40.0, 40.0, 40.0; 
    B_.resize(6, 6); B_.setZero();
    B_.diagonal() << 1.0, 1.0, 1.0, 0.8, 0.8, 0.8;
    tau_f_prev_ = Eigen::VectorXd::Zero(6);
    tau_j_prev_ = Eigen::VectorXd::Zero(6);    
    /* JTS initialization init */
    // robotNominal_ = std::make_unique<ROBOT>(robot);
    jts_initialized_ = false;
    jts_init_cnt_ = 1;
    avg_cnt_ = static_cast<int>(2.0 / _delT);
    tau_j_bias_.setZero();
    /* JTS initialization init end */
        /* Friction Observer initialization init */
    fo_init_status_ = 0;
    dK_L_vec_.setZero();
    fo_init_duration_ = 2.0;
    fo_init_cnt_ = 0;
    fo_init_cnt_total_ = static_cast<int>(fo_init_duration_ / _delT);
    /* Friction Observer initialization init end */

    initializeNominalRobot(robot);

    // temp
    save_integral_.resize(6);
    save_integral_.setZero();
    tau_ext_.resize(6);
    tau_ext_.setZero();
    
    /* Friction Observer initialization reset end */
    std::cout << "========== RESET HAS CALLED ==========" << std::endl;
}

// temp
template<typename ROBOT>
Eigen::VectorXd TaskPDFrictionObserver<ROBOT>::getTauExt(ROBOT &robot, const LieGroup::Vector3D &gravDir, const JointVec & tau_j)
{
    //momentum based observer
    // Eigen::MatrixXd coriolis = this->getCoriolisMatrix(robot_state);
    // Eigen::MatrixXd mass_matrix = this->getMassMatrix(robot_state);
    // Eigen::VectorXd gravity = this->getGravity(robot_state);

    JointMat coriolis; coriolis.setZero(); // coriolis.resize(6,6);
    JointMat mass_matrix; mass_matrix.setZero(); // mass_matrix.resize(6,6);
    JointVec gravity; gravity.setZero(); // gravity.resize(6);
    JointVec q = robot.q();
    JointVec dq = robot.qdot();

    robot.computeDynamicsParams(gravDir, mass_matrix, coriolis, gravity);

    //for observer
    Eigen::MatrixXd KI(6,6); // diagonal gain matrix
    Eigen::VectorXd generalized_momentum(6); // generalized momentum
    KI.setZero();
    KI.setIdentity();
    KI = KI*200;
    generalized_momentum = mass_matrix * dq;
    
    //if starting generalized vel is 0
    Eigen::VectorXd temp_int = _delT*(tau_j + coriolis.transpose()*dq - gravity + tau_ext_);
    Eigen::VectorXd tau_ext = KI*(generalized_momentum -save_integral_ - temp_int);
    save_integral_ = save_integral_ + temp_int;
    tau_ext_ = tau_ext;
    
    return tau_ext;
}


template<typename ROBOT>
void TaskPDFrictionObserver<ROBOT>::compute(ROBOT &robot, const LieGroup::Vector3D &gravDir,
                                               const MotionData &motionData, ControlData &controlData)
{
    // ---------------------------------------------------------------------------------------  My Custom controller  --------------------------------------------------------------
    robot.idyn_gravity(gravDir);
    JointVec gravity_compensation_torque = robot.tau();

    JointVec control_motor_torque; control_motor_torque.setZero();
    JointVec friction_compensation_torque; friction_compensation_torque.setZero();
    JointVec PDinput; PDinput.setZero();
    JointVec new_tau; new_tau.setZero();
    
    Kp_ = this->gain3;
    Kd_ = this->gain4;
    J_lpf_vec_ = this->gain0;
    B_vec_ = this->gain8;
    PW_ = this->gain9;
    Lp_vec_ = this->gain6;

    if (Lp_vec_.minCoeff() > 0)
        Lp_.diagonal() << Lp_vec_;
    if (B_vec_.minCoeff() > 0)
        B_.diagonal() << B_vec_;
    
    const double pw_target_ = 6.29;
    const double pw_tol_ = 1e-6;
    if  (std::abs(PW_[0] - pw_target_) > pw_tol_) {  
        controlData.controlTorque.tau = gravity_compensation_torque;
        if (std::abs(PW_[0]) > pw_tol_)
            std::cout << "PW error" << std::endl;
        return;
    }
        
    if (is_soft_estop_ || std::abs(robot.qdot().maxCoeff()) > 1.5) {
        controlData.controlTorque.tau = gravity_compensation_torque;
        is_soft_estop_ = true;
        std::cout << "EMERGENCY!!! : qdot exceeded" << std::endl;
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
            theta_n_ = robot.q();
            dtheta_n_ = robot.qdot();
            std::cout<<"Joint torque sensor bias is initialized"<<std::endl;
            std::cout<<"bias : " << tau_j_bias_.transpose()<<std::endl;
        }
        new_tau = gravity_compensation_torque;
        
    }
    /* JTS initialization end */
    // -------------------------------- Task CONTROLLER -------------------------------------------------------
    else {
        tau_j_ = controlData.controlTorque.tauJTS;
        tau_j_(1) = - controlData.controlTorque.tauJTS(1);
        tau_j_ -= tau_j_bias_;   // eliminate bias 

        Kp_task_vec_ = this->gain1;
        Kd_task_vec_ = this->gain2;
        if (Kp_task_vec_.minCoeff() > 0)
            Kp_task_.diagonal() << Kp_task_vec_;
        if (Kd_task_vec_.minCoeff() > 0)
            Kd_task_.diagonal() << Kd_task_vec_;

        _robotNominal->computeDynamicsParams(gravDir, _Mn, _Cn, _gn);
        robot.idyn_gravity(gravDir);
        robot.computeFK(tpos_, tvel_);
        robot.computeJacobian(tpos_, tvel_, J_, Jdot_);

        _robotNominal->idyn_gravity(gravDir);
        _robotNominal->computeFK(tposNom_, tvelNom_);
        _robotNominal->computeJacobian(tposNom_, tvelNom_, JNom_, JdotNom_);

        tau_grav_ = robot.tau();
        tau_gravNom_ = _robotNominal->tau();

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
        ExtendedVelocity velDesired; 
        velDesired.v()  = vel_desired_linear;   
        velDesired.w().setZero();

        _robotNominal->computeTaskErr(tposNom_, tvelNom_, posDesired, velDesired, e, edot);

        JointVec tau_task = JNom_.transpose() * ( Kp_task_  * e + Kd_task_ * edot) + gravity_compensation_torque; 

        // Calculate PD Observer // 
        if (this->gain5.minCoeff() > 0) {
            switch (fo_init_status_) {
                case 0:
                    dK_L_vec_ = this->gain5 / (fo_init_duration_ / _delT);
                    fo_init_cnt_ = 0;
                    fo_init_status_ = 1;
                    break;
                case 1:
                    L_vec_ += dK_L_vec_;
                    fo_init_cnt_++;
                    if (fo_init_cnt_ > fo_init_cnt_total_) {
                        fo_init_status_ = 2;
                    }
                    break;
                case 2:
                    L_vec_ = this->gain5;
                    break;
                default:
                    L_vec_.setZero();
            }
        }
        L_.diagonal() << L_vec_;
        JointVec de_nr = _robotNominal->qdot() - robot.qdot();
        JointVec e_nr = _robotNominal->q() - robot.q();
        friction_compensation_torque = -B_ * L_ * (de_nr + Lp_ * e_nr);
        
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6,6);
        Eigen::MatrixXd Beta(6, 6); Beta.setZero();
        Beta.diagonal() = (- J_lpf_vec_ * _delT).array().exp();
        tau_j_lpf_ = Beta * tau_j_prev_ + (I - Beta) * tau_j_;
        tau_j_prev_ = tau_j_lpf_;

        JointVec qddotNom ; qddotNom.setZero();
        qddotNom = B_.inverse() * ( tau_task - tau_j_lpf_ );   
        _robotNominal->qdot() += _delT*qddotNom;
        _robotNominal->q() += _delT*_robotNominal->qdot();
        _robotNominal->update();
        _robotNominal->updateJacobian();

        JointVec error_; error_ = e;
        Eigen::Vector3d toolpos; toolpos = tpos_.r();
        Eigen::Vector3d desired_toolpos; desired_toolpos = posDesired.r();
        Eigen::Vector3d toolposNom; toolposNom = tposNom_.r();

        for (int i = 0; i < 6; i++) {   
            file_ << error_(i) << ",";
        }
        for (int i = 0; i < 3; i++) {   
            file_ << toolpos(i) << ",";
        }
        for (int i = 0; i < 3; i++) {   
            file_ << desired_toolpos(i) << ",";
        }
        for (int i = 0; i < 3; i++) {   
            file_ << vel_desired_linear(i) << ",";
        }
        file_ << std::endl;

        new_tau = tau_task - friction_compensation_torque;
        // new_tau = gravity_compensation_torque;
    }
    
    currentT_ += _delT;
    controlData.controlTorque.tau = new_tau;
}
template<typename ROBOT>
void TaskPDFrictionObserver<ROBOT>::initializeNominalRobot(ROBOT & robot)
{
    /**< Initialize nominal robot with the actual robot state and update */
    _robotNominal->setTtarget(robot.Ttarget().R(), robot.Ttarget().r());
    _robotNominal->setTReference(robot.Tref().R(), robot.Tref().r());
    _robotNominal->q() = robot.q();
    _robotNominal->qdot() = robot.qdot();
    _robotNominal->update();
    _robotNominal->updateJacobian();
}

template<typename ROBOT>
Eigen::Vector3d TaskPDFrictionObserver<ROBOT>::computeQuinticSpline(const Eigen::Vector3d& p0, const Eigen::Vector3d& pf,
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
Eigen::Vector3d TaskPDFrictionObserver<ROBOT>::computeQuinticSplineVelocity(const Eigen::Vector3d& p0, const Eigen::Vector3d& pf,
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
        POCO_EXPORT_CLASS(TaskPDFrictionObserverCreator)
POCO_END_MANIFEST

