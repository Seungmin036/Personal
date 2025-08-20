#include "CustomPdJointController.h"


template<typename ROBOT>
void CustomPdJointController<ROBOT>::initialize(ROBOT &robot, double delt)
{
    /**< Initialization logic for the controller */
}

template<typename ROBOT>
void CustomPdJointController<ROBOT>::reset(ROBOT &robot)
{
    /**< Reset logic for the controller */
}

template<typename ROBOT>
void CustomPdJointController<ROBOT>::compute(ROBOT &robot, const LieGroup::Vector3D &gravDir,
                                             const MotionData &motionData, ControlData &controlData)
{
    /**< Compute gravitational torque based on the robot's current configuration and gravity direction */
    robot.idyn_gravity(gravDir);
    JointVec tauGrav = robot.tau();

    /**< Calculate the PD control torque */
    JointVec tauPD = _kp.cwiseProduct(motionData.motionPoint.qd - robot.q()) - _kv.cwiseProduct(robot.qdot());

    /**< Combine PD and gravitational torques for the final control command */
    controlData.controlTorque.tau = tauPD + tauGrav;
}

/**< Manifest to allow dynamic loading of the control algorithm */
POCO_BEGIN_MANIFEST(NRMKControl::ControlAlgorithmCreator)
        POCO_EXPORT_CLASS(CustomPdJointControllerCreator)
POCO_END_MANIFEST