

#include "CustomAdmittanceJointController.h"


template<typename ROBOT>
void CustomAdmittanceJointController<ROBOT>::initialize(ROBOT &robot, double delt)
{
    /**< Initialization logic for the controller */
}

template<typename ROBOT>
void CustomAdmittanceJointController<ROBOT>::reset(ROBOT &robot)
{
    /**< Reset logic for the controller */
}

template<typename ROBOT>
void CustomAdmittanceJointController<ROBOT>::compute(ROBOT &robot, const LieGroup::Vector3D &gravDir,
                                             const MotionData &motionData, ControlData &controlData)
{
    /**< Computation logic for the controller */
}

/**< Manifest to allow dynamic loading of the control algorithm */
POCO_BEGIN_MANIFEST(NRMKControl::ControlAlgorithmCreator)
POCO_EXPORT_CLASS(CustomAdmittanceJointControllerCreator)
POCO_END_MANIFEST