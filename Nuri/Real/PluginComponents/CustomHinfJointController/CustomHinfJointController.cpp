//
// Created by Meseret on 2/3/2024.
//

#include "CustomHinfJointController.h"

template<typename ROBOT>
void CustomHinfJointController<ROBOT>::initialize(ROBOT &robot, double delt)
{
    /**< Initialize the H-infinity controller with the given time step */
    robot.initHinfController(delt);

    /**< Reset the integral terms in the H-inifinity control algorithm */
    robot.resetHinfController();

}

template<typename ROBOT>
void CustomHinfJointController<ROBOT>::reset(ROBOT& robot)
{
    /**< Reset the integral terms in the H-inifinity control algorithm */
    robot.resetHinfController();
}

template<typename ROBOT>
void CustomHinfJointController<ROBOT>::compute(ROBOT &robot, const LieGroup::Vector3D &gravDir,
                                               const MotionData &motionData, ControlData &controlData)
{
    /**< Get the updated gains */
    _kp = this->gain0;
    _kv = this->gain1;
    _ki = this->gain2;

    
    /**< Set the H-infinity control gains */
    robot.setHinfControlGain(_kp, _kv, _ki, 1); // Mode: joint space = 1, task space = 2.

    /**< Compute the control torque applying H-infinity control algorithm */
    robot.HinfController(gravDir, motionData.motionPoint.qd, motionData.motionPoint.qdotd, motionData.motionPoint.qddotd);
    std::cout << "robot.tau() uu" << std::endl << robot.tau() << std::endl;
    /**< Store the computed torque in the control data structure, and will be used as a target torque sent to the robot's joint driver */
    controlData.controlTorque.tau = robot.tau();
}

/**< Manifest to allow dynamic loading of the control algorithm */
POCO_BEGIN_MANIFEST(NRMKControl::ControlAlgorithmCreator)
        POCO_EXPORT_CLASS(CustomHinfJointControllerCreator)
POCO_END_MANIFEST
