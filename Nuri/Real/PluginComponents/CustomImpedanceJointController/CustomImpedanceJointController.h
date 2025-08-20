

#pragma once

// Necessary includes
#include <NRMKFramework/AbstractComponent/ControlAlgorithm.h>
#include <NRMKFramework/GlobalDefinitions.h>
#include <Controller/PositionController.h>

#include <Poco/ClassLibrary.h>
#include <Poco/ClassLoader.h>
#include <Poco/MetaObject.h>

template<typename ROBOT>
class CustomImpedanceJointController : public NRMKControl::ControlAlgorithm<ROBOT>
{
    // Type definitions for clarity and ease of maintenance
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
    typedef typename ROBOT::ExtendedTaskJacobian ExtendedTaskJacobian;


public:
    // Constructor and member functions
    CustomImpedanceJointController() = default;
    virtual void initialize(ROBOT& robot, double delt) override;
    virtual void reset(ROBOT& robot) override;
    virtual void compute(ROBOT& robot, const LieGroup::Vector3D& gravityDir,
                         const MotionData& motionData, ControlData& controlData) override;


    void initializeNominalRobot(ROBOT & robot);
    void _readFTsensor(ROBOT & robot, ExtendedForce& extForce);

private:
    /**< declaration of a unique pointer to an object of type ROBOT */
    std::unique_ptr<ROBOT> _robotNominal;

    double _delT;

    /**< FT sensor related variables */
    LieGroup::Wrench    _ftBias;
    ExtendedForce    _ftSensor;
    LieGroup::Wrench    _ftTransformed;
    LieGroup::Rotation  _ft_sensor_orientation;

    JointMat            _Mn, _Cn, _Mhat;
    JointVec            _gn;

    ExtendedPosition _tpos, _tposNom;
    ExtendedVelocity _tvel, _tvelNom;
    ExtendedTaskJacobian _J, _JNom;
    ExtendedTaskJacobian _Jdot, _JdotNom;

    JointVec _tauGrav, _tauGravNom;
    JointVec _tauExt;

    bool _sim_mode;

private:
    JointVec _kp = this->gain3;
    JointVec _kv = this->gain4;
    JointVec _ki = this->gain5;
    JointVec _K = this->gain6;
    JointVec _KC = this->gain7;
    JointVec _KD = this->gain8;
    double _rate = this->gain9[0];

};

// Class creator to facilitate dynamic loading if necessary
class CustomImpedanceJointControllerCreator : public NRMKControl::ControlAlgorithmCreator
{
CONTROL_CREATOR_BODY(CustomImpedanceJointController)
};