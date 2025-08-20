# Custom PD Joint Controller
## Overview

This README outlines the setup and usage of the `CustomPdJointController`, showcasing how to implement a  
straightforward PD (Proportional-Derivative) joint controller with the IndySDK.

## Setup

Implementing a PD controller in IndySDK is straightforward, requiring just a few modifications in the compute 
function as outlined below.
1. **Control Gains Configuration**: Start by defining the gain matrices and setting the PD gains for each joint.
```cpp
    JointMat kp, kd; // PD gain matrices
    ...
    // Assign PD gains for each joint
    for (int i = 0; i < ROBOT::JOINT_DOF; ++i)
    {
        // Customize gains based on individual joint requirements
        switch(i)
        {
            case 0:
            case 1:
            kp(i,i) = 70; kd(i,i) = 55; break;
            case 2:
            kp(i,i) = 40; kd(i,i) = 30; break;
            case 3:
            case 4:
            kp(i,i) = 25; kd(i,i) = 15; break;
            case 5:
            kp(i,i) = 18; kd(i,i) = 3; break;
        }
    }
```
- Proportional and derivative gains are configured individually for each joint.
- The gains provided here serve as examples; it is recommended that users adjust the PD gains as necessary for 
  optimal control performance.
2. **Gravity Torque**: Utilize the IndySDK API to calculate the gravitational torque for each joint based on the 
   current robot configuration. The code snippet below demonstrates how to employ the IndySDK API for this purpose.
```cpp
    robot.idyn_gravity(gravDir);
    tauGrav = robot.tau();
```
3. **PD Control Torque**: Compute the PD control torque using the current and desired joint positions and velocities.
```cpp
     // Compute the PD control torque
    tauPD = kp*(qDesired - robot.q()) - kd*robot.qdot();
```
This approach simplifies the implementation of a PD control system, ensuring effective joint control with 
customizable gain settings for each joint, supported by the IndySDK's computational capabilities for 
computing gravity torque.


## Execution
- After build and install (refer [build and install]), copy `CustomPdJointControllerCreator.comp` from `..
  /IndyDeployment/PluginComponents/`
  to `/home/user/release/IndyDeployment/PluginComponents` directory on the target PC (STEP)
- Modify `Components.json` file located at `/home/user/release/IndyConfigurations/Cobot/Plugins/` for component  
  specification
```json
{
    "JointController": "CustomPdJointControllerCreator",
    "TaskController": "CustomTaskControlCreator"
}
```

[//]: # (- Execute the following command to operate the robot &#40;if the robot is runing, stop and restart&#41;)
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
- Make sure the correct mode is set for effective control
- The `idyn_gravity` API function calculates the gravitational torque according to the robot's present configuration, allowing the torque to be accessed via `robot.tau()`.

## Additional Information
For detailed plugin components development guidance, refer to the [Tutorial](http://docs.neuromeka.com/3.2.0/en/IndySDK/section3/) section. 
This README assumes familiarity with the IndySDK's component templates and focuses on illustrating 
implementation of simple PD controller in IndySDK.
