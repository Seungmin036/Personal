
#include "CustomHinfTaskController.h"

template<typename ROBOT>
void CustomHinfTaskController<ROBOT>::initialize(ROBOT &robot, double delt)
{
    /**< Initialize the H-infinity controller with the given time step */
    robot.initHinfController(delt);


    /**< Reset the integral terms in the H-inifinity control algorithm */
    robot.resetHinfController();

    robot.initTaskErr(delt);
}

template<typename ROBOT>
void CustomHinfTaskController<ROBOT>::reset(ROBOT & robot)
{
    /**< Reset the integral terms in the H-inifinity control algorithm */
    robot.resetHinfController();
}

template<typename ROBOT>
void CustomHinfTaskController<ROBOT>::compute(ROBOT &robot, const LieGroup::Vector3D &gravDir,
                                             const MotionData &motionData, ControlData &controlData)
{
    /**< Get the updated gains */
    _kp = this->gain3;
    _kv = this->gain4;
    _ki = this->gain5;

    /**< Set the H-infinity control gains */
    robot.setHinfControlGain(_kp, _kv, _ki, 2); //mode: joint space = 1, task space = 2

    /**< Compute the control torque applying H-infinity control algorithm */
    robot.HinfController(gravDir, motionData.motionPoint.pd, motionData.motionPoint.pdotd, motionData.motionPoint.pddotd);

    /**< Store the computed torque in the control data structure, and will be used as a target torque sent to the robot's joint driver */
    controlData.controlTorque.tau = robot.tau();
}

// Manifest to allow dynamic loading of the control algorithm
POCO_BEGIN_MANIFEST(NRMKControl::ControlAlgorithmCreator)
        POCO_EXPORT_CLASS(CustomHinfTaskControllerCreator)
POCO_END_MANIFEST