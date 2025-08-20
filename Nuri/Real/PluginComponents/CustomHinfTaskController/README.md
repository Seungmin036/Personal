# Custom $\mathcal{H}_\infty$ Task Controller

## Overview
This README outlines the setup and usage of the `CustomHinfTaskController`, demonstrating the application of  the 
`HinfController` and related APIs within the IndySDK for custom control strategies.

## Setup
1. **Control Gains Adjustment**: The control gains configuration includes ten different vectors, each sized according to the robot's degrees of freedom (DOF). Users can conveniently modify these gains using the gRPC Protocol and integrate them into their controller algorithm implementation.

```python
    gain0 = [0, 0, 0,0, 0, 0]
    gain1 = [0, 0, 0, 0, 0, 0]
    gain2 = [0, 0, 0, 0, 0, 0]
    gain3 = [100, 100, 100,100, 100, 100]
    gain4 = [20, 20, 20, 20, 20, 20]
    gain5 = [600, 600, 450, 350, 350, 350]
    
    config.SetCustomControlGain6(gain0=gain0, gain1=gain1, gain2=gain2, gain3=gain3, gain4=gain4, gain5=gain5)
```

This example demonstrates the use case as follows:
We assign `gain3`, `gain4`, and `gain5` to `_kp`, `_kv`, and `_ki` variables defined in the header file.

```c++
private:
    JointVec _kp = this->gain3;
    JointVec _kv = this->gain4;
    JointVec _ki = this->gain5;
```

Subsequently, to take advantage of updating gains using gRPC Protocols, we update these gains in the compute function and utilize them in the control algorithm.

```c++
    /**< Get the updated gains */
    _kp = this->gain3;
    _kv = this->gain4;
    _ki = this->gain5;
```
2. **Initialization and Reset**: Upon the `initialize` function's invocation during controller instantiation, the 
   integral error is reset, and the sampling time (delt) is established for error integral computation.
```cpp
    robot.initHinfController(delt);
    robot.resetHinfController();
```
3. **Control Mode Selection**: Use `setHinfControlGain` in the source code to apply gains, specifying mode 1  
   for joint space or mode 2 for task space control.
```cpp
    robot.setHinfControlGain(_kp, _kv, _ki, 2); //mode: joint space = 1, task space = 2
```


## Execution
- After build and install (refer [build and install]), copy `CustomHinfTaskControllerCreator.comp` from `..
  /IndyDeployment/PluginComponents/` 
  to `/home/user/release/IndyDeployment/PluginComponents` directory on the target PC (STEP)
- Modify `Components.json` file located at `/home/user/release/IndyConfigurations/Cobot/Plugins/` for component  
  specification
```json
{
    "JointController": "CustomJointControlCreator",
    "TaskController": "CustomHinfTaskControllerCreator"
}
```
- Prior to changing the control mode to your customized control algorithm, ensure that you appropriately set the gain values corresponding to the gains you intend to utilize. By default, all gains are initialized to zero. The following example demonstrates how to set gains using the gRPC Protocol for this scenario.
```python
    gain0 = [0, 0, 0,0, 0, 0]
    gain1 = [0, 0, 0, 0, 0, 0]
    gain2 = [0, 0, 0, 0, 0, 0]
    gain3 = [100, 100, 100,100, 100, 100]
    gain4 = [20, 20, 20, 20, 20, 20]
    gain5 = [600, 600, 450, 350, 350, 350]
    
    config.SetCustomControlGain6(gain0=gain0, gain1=gain1, gain2=gain2, gain3=gain3, gain4=gain4, gain5=gain5)
```
- To verify whether the gain has been successfully set or not, you can use the get function as follows:
```python
    config.GetCustomControlGain()
```
The output will be:
```python
{'gain0': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
 'gain1': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
 'gain2': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
 'gain3': [100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
 'gain4': [20.0, 20.0, 20.0, 20.0, 20.0, 20.0],
 'gain5': [600.0, 600.0, 450.0, 350.0, 350.0, 350.0],
 'gain6': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
 'gain7': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
 'gain8': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
 'gain9': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
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
- The `HinfController` function computes the desired torque,  accessible via `robot.tau()`, based on the 
  current robot state and desired task trajectories.

## Additional Information
For detailed plugin components development guidance, refer to the [Tutorial](http://docs.neuromeka.com/3.2.0/en/IndySDK/section3/) section. This README assumes familiarity 
with the IndySDK's component templates and focuses on deploying `HinfController` API and associated functionalities.
