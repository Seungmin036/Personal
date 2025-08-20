
#pragma once

// Necessary includes
#include <NRMKFramework/AbstractComponent/ControlAlgorithm.h>
#include <NRMKFramework/GlobalDefinitions.h>
#include <Controller/PositionController.h>

#include <Poco/ClassLibrary.h>
#include <Poco/ClassLoader.h>
#include <Poco/MetaObject.h>

template<typename ROBOT>
class CustomHinfTaskController : public NRMKControl::ControlAlgorithm<ROBOT>
{
    // Type definitions for clarity and ease of maintenance
    typedef NRMKControl::ControlAlgorithm<ROBOT> AlgorithmBase;
    typedef typename AlgorithmBase::ControlGains ControlGains;
    typedef typename NRMKControl::ControlAlgorithm<ROBOT>::MotionData MotionData;
    typedef typename NRMKControl::ControlAlgorithm<ROBOT>::ControlData ControlData;
    typedef typename ROBOT::JointVec JointVec;
    typedef typename ROBOT::JointMat JointMat;

public:
    // Constructor and member functions
    CustomHinfTaskController() = default;
    virtual void initialize(ROBOT& robot, double delt) override;
    virtual void reset(ROBOT& robot) override;
    virtual void compute(ROBOT& robot, const LieGroup::Vector3D& gravityDir,
                         const MotionData& motionData, ControlData& controlData) override;

private:
    JointVec _kp = this->gain3;
    JointVec _kv = this->gain4;
    JointVec _ki = this->gain5;
};

// Class creator to facilitate dynamic loading if necessary
class CustomHinfTaskControllerCreator : public NRMKControl::ControlAlgorithmCreator
{
CONTROL_CREATOR_BODY(CustomHinfTaskController)
};
