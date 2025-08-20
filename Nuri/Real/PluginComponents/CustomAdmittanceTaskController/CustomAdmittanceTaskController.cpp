

#include "CustomAdmittanceTaskController.h"

template<typename ROBOT>
void CustomAdmittanceTaskController<ROBOT>::initialize(ROBOT &robot, double delt)
{
    /**< Create a new 'ROBOT' object as a copy of the 'robot'. 'ROBOT' is copy-constructible. */
    _robotNominal = std::make_unique<ROBOT>(robot);

    /**< align FT sensor coordinate with robot target body coordinates */
    _ft_sensor_orientation = LieGroup::Rotation(0,-1,0,1,0,0,0,0,1);
    robot.setTFTSensor(_ft_sensor_orientation, LieGroup::Displacement::Zero());

    /**< Initialize position controller */
    robot.initForwardDynController(delt);
    robot.resetForwardDynController();

    /**< initialization */
    _tposProxy.setZero();
    _tvelProxy.setZero();
    _taccProxy.setZero();

    /**< update sampling time */
    _delT = delt;

    /**< initialize task-space error */
    robot.initTaskErr(delt);
}

template<typename ROBOT>
void CustomAdmittanceTaskController<ROBOT>::reset(ROBOT& robot)
{
    /**< reset nominal robot */
    initializeNominalRobot(robot);

    /**< reset admittance trajectory */
    robot.resetAdmittanceTraj(_delT);
    robot.resetForwardDynController();
}

template<typename ROBOT>
void CustomAdmittanceTaskController<ROBOT>::compute(ROBOT &robot, const LieGroup::Vector3D &gravDir,
                                             const MotionData &motionData, ControlData &controlData)
{
    /**< Retrieve the updated gains by gRPC protocols */
    _kp = this->gain3;
    _kv = this->gain4;
    _ki = this->gain5;
    _mass_xyz = this->gain6[0];
    _mass_uvw = this->gain6[1];
    _stiffness_xyz = this->gain6[2];
    _stiffness_uvw = this->gain6[3];

    LOG_INFO("_mass_xyz: %.2f, _mass_uvw: %.2f, _stiffness_xyz: %.2f, _stiffness_uvw: %.2f",_mass_xyz,
                                                                                            _mass_uvw,
                                                                                            _stiffness_xyz,
                                                                                            _stiffness_uvw);

    /**< Nun-constant reference (Workaround to update desired Task desired trajectories) */
    auto& nonConstMotionData = const_cast<MotionData&>(motionData);

    /**< read FT sensor and  */
    ExtendedForce &force = controlData.extForce;

    _readFTsensor(robot, force);

    /**< change FT sensor direction x,y,z,u,v,w to u,v,w,x,y,z */
    for (int i=0; i<3; i++) _ftTransformed[i] = _ftSensor[i+3];
    for (int i=0; i<3; i++) _ftTransformed[i+3] = _ftSensor[i];

    /**< calculate admittance trajectory based on desired trajectories and ft sensor values */
    ExtendedPosition posDesired; posDesired.setZero();
    posDesired.R()  = nonConstMotionData.motionPoint.pd.R();
    posDesired.r()  = nonConstMotionData.motionPoint.pd.r();

    ExtendedVelocity velDesired; velDesired.setZero();
    velDesired.v()  = nonConstMotionData.motionPoint.pdotd.v();
    velDesired.w()  = nonConstMotionData.motionPoint.pdotd.w();

    ExtendedAcceleration accDesired ; accDesired.setZero();
    accDesired.v()  = nonConstMotionData.motionPoint.pddotd.v();
    accDesired.w()  = nonConstMotionData.motionPoint.pddotd.w();

    /**< set control gains and update admittance trajectory to controllers */
    robot.setForwardDynControlGain(_kp, _kv, _ki, _mass_xyz, _mass_uvw, _stiffness_xyz, _stiffness_uvw);
    robot.updateAdmittanceTraj(posDesired,
                               velDesired,
                               accDesired,
                               _tposProxy,
                               _tvelProxy,
                               _taccProxy,
                               _ftTransformed,  // filtered
                               _ftTransformed,
                               1);

    /**< update admittance trajectory */
    nonConstMotionData.motionPoint.pd.R() = _tposProxy.R();
    nonConstMotionData.motionPoint.pd.r() = _tposProxy.r();
    nonConstMotionData.motionPoint.pdotd.w() = _tvelProxy.w();
    nonConstMotionData.motionPoint.pdotd.v() = _tvelProxy.v();
    nonConstMotionData.motionPoint.pddotd.w() = _taccProxy.w();
    nonConstMotionData.motionPoint.pddotd.v() = _taccProxy.v();

    nonConstMotionData.motionPoint.qd = _robotNominal->q();
    nonConstMotionData.motionPoint.qdotd = _robotNominal->qdot();

    robot.ForwardDynController(*_robotNominal, _Mn, _Cn, _gn, _tposProxy, _tvelProxy, gravDir);

    /**< computed desired torque */
    controlData.controlTorque.tau = robot.tau();
}

template<typename ROBOT>
void CustomAdmittanceTaskController<ROBOT>::initializeNominalRobot(ROBOT & robot)
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
void CustomAdmittanceTaskController<ROBOT>::_readFTsensor(ROBOT & robot, ExtendedForce& extForce)
{
    ExtendedPosition tposT;
    ExtendedVelocity tvelT;
    robot.computeFK(tposT, tvelT);

    LieGroup::Wrench _ftSensorTemp; _ftSensorTemp.setZero();
    _ftSensorTemp.f() = extForce.f();
    _ftSensorTemp.n() = extForce.n();
    _ftSensorTemp = _ftSensorTemp - _ftBias;

    LieGroup::HTransform T_ft_target = robot.Ttarget().icascade(robot.Tft());
    LieGroup::Wrench _ftSensorTemp2 = T_ft_target.itransform(_ftSensorTemp);
    LieGroup::Wrench _ftSensorTransformed = LieGroup::Wrench(tposT.R()*_ftSensorTemp2.f(), _ftSensorTemp2.n());

    for (int i=0; i<3; i++) _ftSensor[i] = _ftSensorTransformed[i+3];
    for (int i=0; i<3; i++) _ftSensor[i+3] = _ftSensorTransformed[i];
}

/**< Manifest to allow dynamic loading of the control algorithm */
POCO_BEGIN_MANIFEST(NRMKControl::ControlAlgorithmCreator)
        POCO_EXPORT_CLASS(CustomAdmittanceTaskControllerCreator)
POCO_END_MANIFEST