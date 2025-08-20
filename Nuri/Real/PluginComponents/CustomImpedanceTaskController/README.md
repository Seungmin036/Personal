# Custom Task Space Impedance Control

## Overview
This README outlines the setup and usage of the `CustomImpedanceTaskController`, demonstrating the application of 
different APIs within the IndySDK for custom control strategies.

## Setup
1. **Control Gains Adjustment**: The control gains configuration includes ten different vectors, each sized according to the robot's degrees of freedom (DOF). Users can conveniently modify these gains using the gRPC Protocol and integrate them into their controller algorithm implementation.

```python
simMode = True
if(simMode):
    # Joint controller 
    gain0 = [100, 100, 100,100, 100, 100] #kp
    gain1 = [20, 20, 20, 20, 20, 20] #kv
    gain2 = [0, 0, 0, 0, 0, 0] #ki
    # Impedance control (Task space)
    gain3 = [150, 150, 80, 25, 25, 18] #kp
    gain4 = [25, 25, 18, 3, 3, 1.5] #kv
    gain5 = [0, 0, 0, 0, 0, 0] #ki
    gain6 = [0.5, 0.5, 0.5, 0.5, 0.5, 0.02] #K
    gain7 = [80, 80, 40, 25, 25, 25] #KC
    gain8 = [55, 55, 30, 15, 15, 15] #KD
    gain9 = [4, 0, 0, 0, 0, 0] #rate
    gainzero = [0, 0, 0, 0, 0, 0]


else:
    # Joint controller 
    gain0 = [100, 100, 100,100, 100, 100] #kp
    gain1 = [20, 20, 20, 20, 20, 20] #kv
    gain2 = [0, 0, 0, 0, 0, 0] #ki
    # Impedance control (Task space)
    gain3 = [150, 150, 80, 25, 25, 18] #kp
    gain4 = [25, 25, 18, 3, 3, 1.5] #kv
    gain5 = [0, 0, 0, 0, 0, 0] #ki
    gain6 = [1, 1, 1, 1, 1, 0.05] #K
    gain7 = [80, 80, 40, 25, 25, 25] #KC
    gain8 = [55, 55, 30, 15, 15, 15] #KD
    gain9 = [4, 0, 0, 0, 0, 0] #rate
    gainzero = [0, 0, 0, 0, 0, 0]




config.SetCustomControlGain(gain0=gain0, gain1=gain1, gain2=gain2, gain3=gain3, gain4=gain4, gain5=gain5, gain6=gain6, gain7=gain7, gain8=gain8, gain9=gain9)
```

This example demonstrates the use case as follows:
We assign `gain3`, `gain4`, `gain6`, `gain7` , `gain8` and `gain9[0]` to `_kp`, `_kv`, `_K`, `_KC`, `_KD` and `_rate` variables defined in the header file.

```c++
private:
    JointVec _kp = this->gain3;
    JointVec _kv = this->gain4;
    JointVec _ki = this->gain5;
    JointVec _K = this->gain6;
    JointVec _KC = this->gain7;
    JointVec _KD = this->gain8;
    double _rate = this->gain9[0];
```

Subsequently, to take advantage of updating gains using gRPC Protocols, we update these gains in the compute function and utilize them in the control algorithm.

```c++
    /**< Get the updated gains */
    _kp = this->gain3;
    _kv = this->gain4;
    _ki = this->gain5;
    _K = this->gain6;
    _KC = this->gain7;
    _KD = this->gain8;
    _rate = this->gain9[0];
```
2. **Initialization and Reset**: Upon the `initialize` function's invocation during controller instantiation, the
   integral error is reset, and the sampling time (delt) is established for error integral computation.
```cpp
    initializeNominalRobot(robot);
```


## Execution
- After build and install (refer [build and install]), copy `CustomImpedanceTaskControllerCreator.comp` from `..
  /IndyDeployment/PluginComponents/`
  to `/home/user/release/IndyDeployment/PluginComponents` directory on the target PC (STEP)
- Modify `Components.json` file located at `/home/user/release/IndyConfigurations/Cobot/Plugins/` for component  
  specification
```json
{
    "JointController": "CustomJointControlCreator",
    "TaskController": "CustomImpedanceTaskControllerCreator"
}
```
- Prior to changing the control mode to your customized control algorithm, ensure that you appropriately set the gain values corresponding to the gains you intend to utilize. By default, all gains are initialized to zero. The following example demonstrates how to set gains using the gRPC Protocol for this scenario.
```python
simMode = True
if(simMode):
    # Joint controller 
    gain0 = [100, 100, 100,100, 100, 100] #kp
    gain1 = [20, 20, 20, 20, 20, 20] #kv
    gain2 = [0, 0, 0, 0, 0, 0] #ki
    # Impedance control (Task space)
    gain3 = [150, 150, 80, 25, 25, 18] #kp
    gain4 = [25, 25, 18, 3, 3, 1.5] #kv
    gain5 = [0, 0, 0, 0, 0, 0] #ki
    gain6 = [0.5, 0.5, 0.5, 0.5, 0.5, 0.02] #K
    gain7 = [80, 80, 40, 25, 25, 25] #KC
    gain8 = [55, 55, 30, 15, 15, 15] #KD
    gain9 = [4, 0, 0, 0, 0, 0] #rate
    gainzero = [0, 0, 0, 0, 0, 0]


else:
    # Joint controller 
    gain0 = [100, 100, 100,100, 100, 100] #kp
    gain1 = [20, 20, 20, 20, 20, 20] #kv
    gain2 = [0, 0, 0, 0, 0, 0] #ki
    # Impedance control (Task space)
    gain3 = [150, 150, 80, 25, 25, 18] #kp
    gain4 = [25, 25, 18, 3, 3, 1.5] #kv
    gain5 = [0, 0, 0, 0, 0, 0] #ki
    gain6 = [1, 1, 1, 1, 1, 0.05] #K
    gain7 = [80, 80, 40, 25, 25, 25] #KC
    gain8 = [55, 55, 30, 15, 15, 15] #KD
    gain9 = [4, 0, 0, 0, 0, 0] #rate
    gainzero = [0, 0, 0, 0, 0, 0]




config.SetCustomControlGain(gain0=gain0, gain1=gain1, gain2=gain2, gain3=gain3, gain4=gain4, gain5=gain5, gain6=gain6, gain7=gain7, gain8=gain8, gain9=gain9)
```
- To verify whether the gain has been successfully set or not, you can use the get function as follows:
```python
    config.GetCustomControlGain()
```
The output will be:
```python
{'gain0': [100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
 'gain1': [20.0, 20.0, 20.0, 20.0, 20.0, 20.0],
 'gain2': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
 'gain3': [150.0, 150.0, 80.0, 25.0, 25.0, 18.0],
 'gain4': [25.0, 25.0, 18.0, 3.0, 3.0, 1.5],
 'gain5': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
 'gain6': [0.5, 0.5, 0.5, 0.5, 0.5, 0.02],
 'gain7': [80.0, 80.0, 40.0, 25.0, 25.0, 25.0],
 'gain8': [55.0, 55.0, 30.0, 15.0, 15.0, 15.0],
 'gain9': [4.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
```
- Additionally, before changing control mode, ensure that you activate your license using the following procedure:
```python
ControlStub::ActivateIndySDK(SDKLicenseInfo(license_key=key,expire_date=date))
```

- To operate the robot, execute the following command (if it is already running, stop the program and follow this
  process).
```bash
$ cd /home/user/release/IndyDeployment
$ sudo python3 indy_run.py
```
- To test your customized control algorithm, change the control mode using gRPC Protocol. The default control mode  is
  **0**, which corresponds to the default controller. To test your custom control algorithm, set the control mode to **1**.

```python
ControlStub::SetCustomControlMode(IntMode(mode=mode))
```
- To verify that the control mode has been updated, use the `getMode()` method:
```python
mode = ControlStub::GetCustomControlMode(Empty())
```

## Note
- Make sure you activate your license and the correct mode is set for effective control


## Additional Information
For detailed plugin components development guidance, refer to the [Tutorial](http://docs.neuromeka.com/3.2.0/en/IndySDK/section3/) section. This README assumes familiarity
with the IndySDK's component templates and focuses on deploying different APIs.
