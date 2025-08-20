

#pragma once

// Necessary includes
#include <NRMKFramework/AbstractComponent/ControlAlgorithm.h>
#include <NRMKFramework/GlobalDefinitions.h>
#include <Controller/PositionController.h>

#include <Poco/ClassLibrary.h>
#include <Poco/ClassLoader.h>
#include <Poco/MetaObject.h>

template<typename ROBOT>
class CustomAdmittanceTaskController : public NRMKControl::ControlAlgorithm<ROBOT>
{
    /**< Type definitions for clarity and ease of maintenance */
    typedef NRMKControl::ControlAlgorithm<ROBOT> AlgorithmBase;
    typedef typename AlgorithmBase::ControlGains ControlGains;
    typedef typename NRMKControl::ControlAlgorithm<ROBOT>::MotionData MotionData;
    typedef typename NRMKControl::ControlAlgorithm<ROBOT>::ControlData ControlData;
    typedef typename ROBOT::JointVec JointVec;
    typedef typename ROBOT::JointMat JointMat;
    typedef typename ROBOT::TaskPosition  TaskPosition;
    typedef typename ROBOT::TaskVelocity TaskVelocity;
    typedef typename ROBOT::TaskAcceleration TaskAcceleration;
    typedef typename ROBOT::ExtendedPosition ExtendedPosition;
    typedef typename ROBOT::ExtendedVelocity ExtendedVelocity;
    typedef typename ROBOT::ExtendedAcceleration ExtendedAcceleration;
    typedef typename ROBOT::ExtendedForce ExtendedForce;
    typedef LieGroup::Wrench TaskForce;


public:
    /**< Constructor and member functions */
    CustomAdmittanceTaskController() = default;
    virtual void initialize(ROBOT& robot, double delt) override;
    virtual void reset(ROBOT& robot) override;
    virtual void compute(ROBOT& robot, const LieGroup::Vector3D& gravityDir,
                         const MotionData& motionData, ControlData& controlData) override;

    /**< Algorithm specific variable definition */

    /**< FT sensor data reading and coordinate transformation */
    void _readFTsensor(ROBOT & robot, ExtendedForce& extForce);
    void setBias() {_ftBias = _ftSensor;}

    void initializeNominalRobot(ROBOT & robot);

private:
    /**< declaration of a unique pointer to an object of type ROBOT */
    std::unique_ptr<ROBOT> _robotNominal;

private:
    JointVec _kp = this->gain3;
    JointVec _kv = this->gain4;
    JointVec _ki = this->gain5;
    double _mass_xyz = this->gain6[0];
    double _mass_uvw = this->gain6[1];
    double _stiffness_xyz = this->gain6[2];
    double _stiffness_uvw = this->gain6[3];

private:
    /**< FT sensor related variables */
    LieGroup::Wrench    _ftBias;
    LieGroup::Wrench    _ftSensor;
    LieGroup::Wrench    _ftTransformed;
    LieGroup::Rotation  _ft_sensor_orientation;

    double _delT;

    JointMat _Mn, _Cn;
    JointVec _gn;


private:
    /**<reference trajectory */
    ExtendedPosition _tposProxy;
    ExtendedVelocity _tvelProxy;
    ExtendedAcceleration _taccProxy;

private:

};

/**< Class creator to facilitate dynamic loading if necessary */
class CustomAdmittanceTaskControllerCreator : public NRMKControl::ControlAlgorithmCreator
{
CONTROL_CREATOR_BODY(CustomAdmittanceTaskController)
};