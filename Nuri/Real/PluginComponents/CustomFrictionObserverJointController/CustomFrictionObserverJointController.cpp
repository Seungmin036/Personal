
#include "CustomFrictionObserverJointController.h"

template<typename ROBOT>
void CustomFrictionObserverJointController<ROBOT>::initialize(ROBOT &robot, double delt)
{

}

template<typename ROBOT>
void CustomFrictionObserverJointController<ROBOT>::reset(ROBOT &robot)
{
    /**< Reset logic for the controller */

}

template<typename ROBOT>
void CustomFrictionObserverJointController<ROBOT>::compute(ROBOT &robot, const LieGroup::Vector3D &gravDir,
                                                         const MotionData &motionData, ControlData &controlData)
{
    /**< Computation logic for the controller */

}

POCO_BEGIN_MANIFEST(NRMKControl::ControlAlgorithmCreator)
        POCO_EXPORT_CLASS(CustomFrictionObserverJointControllerCreator)
POCO_END_MANIFEST
