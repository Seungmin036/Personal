
#include "CustomComputedTorqueJointController.h"

template<typename ROBOT>
void CustomComputedTorqueJointController<ROBOT>::initialize(ROBOT &robot, double delt)
{
    /**< Create a new 'ROBOT' object as a copy of the 'robot'. 'ROBOT' is copy-constructible. */
    _robotNominal = std::make_unique<ROBOT>(robot);

    _delT = delt;
    _intError.fill(0.0);
}

template<typename ROBOT>
void CustomComputedTorqueJointController<ROBOT>::reset(ROBOT& robot)
{
    initializeNominalRobot(robot);
    _intError.setZero();
}

template<typename ROBOT>
void CustomComputedTorqueJointController<ROBOT>::compute(ROBOT &robot, const LieGroup::Vector3D &gravDir,
                                             const MotionData &motionData, ControlData &controlData)
{
    /**< Get the updated gains by gRPC protocols */
    _kp = this->gain0;
    _kv = this->gain1;
    _ki = this->gain2;

    /**< compute gravity torque */
    robot.idyn_gravity(gravDir);
    JointVec tauGrav = robot.tau();

    JointVec err = motionData.motionPoint.qd -  robot.q();
    JointVec errdot = motionData.motionPoint.qdotd -  robot.qdot();

    /**< compute dynamic parameters */
    _robotNominal->computeDynamicsParams(gravDir, _Mn, _Cn, _gn);

    /**< implement computed controller control algorithm  */
    JointVec tauCTC = _Mn * (motionData.motionPoint.qddotd + _kp.cwiseProduct(err) + _kv.cwiseProduct(errdot) + _ki.cwiseProduct(_intError))
            + _Cn * robot.qdot();

    _intError += err * _delT;

    /**< update nominal robot */
    JointVec qddotNom = _Mn.inverse() * (tauCTC - _Cn * _robotNominal->qdot());
    _robotNominal->q() += _robotNominal->qdot() * _delT;
    _robotNominal->qdot() += qddotNom * _delT;
    _robotNominal->update();
    _robotNominal->updateJacobian();

    controlData.controlTorque.tau = tauCTC + tauGrav;
}

template<typename ROBOT>
void CustomComputedTorqueJointController<ROBOT>::initializeNominalRobot(ROBOT &robot)
{
    /**<  Initialize nominal robot with the actual robot state and update */
    _robotNominal->setTtarget(robot.Ttarget().R(), robot.Ttarget().r());
    _robotNominal->setTReference(robot.Tref().R(), robot.Tref().r());
    _robotNominal->q() = robot.q();
    _robotNominal->qdot() = robot.qdot();
    _robotNominal->update();
    _robotNominal->updateJacobian();
}


/**< Manifest to allow dynamic loading of the control algorithm */
POCO_BEGIN_MANIFEST(NRMKControl::ControlAlgorithmCreator)
        POCO_EXPORT_CLASS(CustomComputedTorqueJointControllerCreator)
POCO_END_MANIFEST