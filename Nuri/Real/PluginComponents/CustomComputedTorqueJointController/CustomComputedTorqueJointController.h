
#pragma once

/**< Necessary includes */
#include <NRMKFramework/AbstractComponent/ControlAlgorithm.h>
#include <NRMKFramework/GlobalDefinitions.h>
#include <Controller/PositionController.h>

#include <Poco/ClassLibrary.h>
#include <Poco/ClassLoader.h>
#include <Poco/MetaObject.h>

template<typename ROBOT>
class CustomComputedTorqueJointController : public NRMKControl::ControlAlgorithm<ROBOT>
{
    /**< Type definitions for clarity and ease of maintenance */
    typedef NRMKControl::ControlAlgorithm<ROBOT> AlgorithmBase;
    typedef typename AlgorithmBase::ControlGains ControlGains;
    typedef typename NRMKControl::ControlAlgorithm<ROBOT>::MotionData MotionData;
    typedef typename NRMKControl::ControlAlgorithm<ROBOT>::ControlData ControlData;
    typedef typename ROBOT::JointVec JointVec;
    typedef typename ROBOT::JointMat JointMat;

public:
    /**< Constructor and member functions */
    CustomComputedTorqueJointController() = default;
    virtual void initialize(ROBOT& robot, double delt) override;
    virtual void reset(ROBOT& robot) override;
    virtual void compute(ROBOT& robot, const LieGroup::Vector3D& gravityDir,
                         const MotionData& motionData, ControlData& controlData) override;

    /**< Algorithm specific variable definition */

    void initializeNominalRobot(ROBOT & robot);

private:
    /**< declaration of a unique pointer to an object of type ROBOT */
    std::unique_ptr<ROBOT> _robotNominal;
    double _delT;

    JointVec _kp = this->gain0;
    JointVec _kv = this->gain1;
    JointVec _ki = this->gain2;
    JointVec _intError;

    JointMat _Mn, _Cn;
    JointVec _gn;

};

/**< Class creator to facilitate dynamic loading if necessary */
class CustomComputedTorqueJointControllerCreator : public NRMKControl::ControlAlgorithmCreator
{
CONTROL_CREATOR_BODY(CustomComputedTorqueJointController)
};
