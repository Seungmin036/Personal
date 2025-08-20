
#include "CustomImpedanceJointController.h"


template<typename ROBOT>
void CustomImpedanceJointController<ROBOT>::initialize(ROBOT &robot, double delt)
{
    /**< Initialization logic for the controller */
    /**< ==================================================================== */

    /**< Create a new 'ROBOT' object as a copy of the 'robot'. 'ROBOT' is copy-constructible. */
    _robotNominal = std::make_unique<ROBOT>(robot);

    /**< align FT sensor coordinate with robot target body coordinates */
    _ft_sensor_orientation = LieGroup::Rotation(0,-1,0,1,0,0,0,0,1);
    robot.setTFTSensor(_ft_sensor_orientation, LieGroup::Displacement::Zero());

    /** update sampling time */
    _delT = delt;

    // initialization
    _J.setZero();
    _Jdot.setZero();
    _Mhat.setZero();
    _tauExt.setZero();
}

template<typename ROBOT>
void CustomImpedanceJointController<ROBOT>::reset(ROBOT &robot)
{
    /**< Reset logic for the controller */
    /**< ==================================================================== */

    /**< reset nominal robot */
    initializeNominalRobot(robot);

    /***< sim_mode/real_mode */
    _sim_mode = true;


}

template<typename ROBOT>
void CustomImpedanceJointController<ROBOT>::compute(ROBOT &robot, const LieGroup::Vector3D &gravDir,
                                             const MotionData &motionData, ControlData &controlData)
{
    /**< Computation logic for the controller */
    /**< ==================================================================== */

    /**< Retrieve the updated gains by gRPC protocols */
    _kp = this->gain3;
    _kv = this->gain4;
    _ki = this->gain5;
    _K = this->gain6;
    _KC = this->gain7;
    _KD = this->gain8;
    _rate = this->gain9[0];


    /**< read FT sensor */
    ExtendedForce &force = controlData.extForce;
    _readFTsensor(robot, force);

    /**< compute dynamic parameters */
    _robotNominal->computeDynamicsParams(gravDir, _Mn, _Cn, _gn);
    _Mhat = _Mn;


    for (int i = 0; i < ROBOT::JOINT_DOF; i++)
    {
        switch(i)
        {
            case 0:
            case 1:
                if(_sim_mode) _Mhat(i,i) = _Mn(i,i);
                else _Mhat(i,i) = _Mn(i,i) + 40;
                break;
            case 2:
                if(_sim_mode) _Mhat(i,i) = _Mn(i,i);
                else _Mhat(i,i) = _Mn(i,i) + 20;
                break;
            case 3:
            case 4:
            case 5:
                if(_sim_mode) _Mhat(i,i) = _Mn(i,i);
                else _Mhat(i,i) = _Mn(i,i) + 10;
                break;
        }
    }

    robot.idyn_gravity(gravDir);
    robot.computeFK(_tpos, _tvel);
    robot.computeJacobian(_tpos, _tvel, _J, _Jdot);

    _robotNominal->idyn_gravity(gravDir);
    _robotNominal->computeFK(_tposNom, _tvelNom);
    _robotNominal->computeJacobian(_tposNom, _tvelNom, _JNom, _JdotNom);

    /**< There is issues with dimension mismatch if we use _ftSensor directly. */
    JointVec temp1FT;
    temp1FT.setZero();
    for (int i = 0; i < 3; i++) {
        temp1FT(i+3,0) = _ftSensor.f()[i];
        temp1FT(i,0)   = _ftSensor.n()[i];
    }
    _tauExt = _J.transpose() * temp1FT;
    _tauGrav = robot.tau();
    _tauGravNom = _robotNominal->tau();

    /**< calculate Impedance Control input */
    JointVec tauImp, tauAux;
    tauImp.setZero();
    tauAux.setZero();

    tauImp =  _kp.cwiseProduct(motionData.motionPoint.qd - _robotNominal->q()) - _kv.cwiseProduct(_robotNominal->qdot());

    JointVec qddotNom, err, err_dot;
    qddotNom.setZero();
    err.setZero();
    err_dot.setZero();

    qddotNom = _Mhat.inverse()*(_K.cwiseProduct(tauImp + _tauExt) - _Cn*_robotNominal->qdot());

    /**< update nominal states */
    _robotNominal->q() += (float) _delT*_robotNominal->qdot();
    _robotNominal->qdot() += (float) _delT*qddotNom;
    _robotNominal->update();
    _robotNominal->updateJacobian();

    /**< define errors between real and nominal robots */
    err = _robotNominal->q() - robot.q();
    err_dot = _robotNominal->qdot() - robot.qdot();

    /**< calculate auxiliary control input */
    tauAux = _rate*_KC.cwiseProduct(err) + _rate*_KD.cwiseProduct(err_dot);


    /**< final robust impedance control input */
    controlData.controlTorque.tau = _tauGrav - _tauExt + _K.cwiseProduct(tauImp + tauAux + _tauExt);
}

template<typename ROBOT>
void CustomImpedanceJointController<ROBOT>::initializeNominalRobot(ROBOT & robot)
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
void CustomImpedanceJointController<ROBOT>::_readFTsensor(ROBOT & robot, ExtendedForce& extForce)
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

    for (int i=0; i<3; i++) _ftSensor.n()[i] = _ftSensorTransformed[i+3];
    for (int i=0; i<3; i++) _ftSensor.f()[i] = _ftSensorTransformed[i];
}

/**< Manifest to allow dynamic loading of the control algorithm */
POCO_BEGIN_MANIFEST(NRMKControl::ControlAlgorithmCreator)
        POCO_EXPORT_CLASS(CustomImpedanceJointControllerCreator)
POCO_END_MANIFEST