# TeleXperience - Robot Joint Position Controller Test

This guide provides instructions on how to test whether your robot joint position controller meets TeleXperience requirements, which involve processing a 100 Hz robot joint position (in radians) command.

To achieve this, you need to create a ROS node that:
1. Publishes joint state feedback.
2. Subscribes to joint position commands.

## 1. Dependencies

- **ROS1 required**
- If you need to use **ROS2**, simply modify the ROS-related code in `joint_control.py` accordingly.

## 2. Publish Joint State Feedback

You need to implement a ROS publisher that publishes your robot's real joint states feedback to the topic: `/io_teleop/joint_states` with message type: `sensor_msgs/JointState`

### Requirements:
- Ensure you populate the following fields in the `JointState` message:
  - `name`: List of joint names.
  - `position`: Corresponding joint positions (in radians).
- The frequency is not strictly restricted, but it should be high enough for real-time monitoring.

## 3. Edit Configuration

Modify the configuration file `config/tool.yml` to specify the joints you want to test control on.

### Example:
```yaml
L_SHOULDER_R:
    type: "sine"
    magnitude: 0.1
    frequency: 0.2
```

### Guidelines:
1. Start with joints near the end of the robot arm and gradually move towards the base.
2. The joint name you specify must exist in the received message from `/io_teleop/joint_states`.
3. The `type` field can have the following values:
   - `sine`: Generates a sinusoidal trajectory.
   - `step`: Moves the joint to a target position abruptly.
   - `ramp`: Moves the joint gradually to a target position.
4. The `magnitude` field represents:
   - The amplitude of the command.
   - **Important:** Start with a small value and gradually increase it while staying within the joint's limit.
5. `frequence`:
   - Lower values result in slower motion. (not for `step`) 

⚠ **Collision avoidance is your responsibility!** Ensure that the test does not cause unintended contact or damage.

## 4. Run the Test

Execute the following command to start the control script:

```
python3 joint_control.py
```

## 5. Subscribe to Joint Position Command

You need to implement a ROS subscriber to receive joint commands from: `/io_teleop/joint_cmd` with message type: `sensor_msgs/JointState`

### Implementation:
- Implement a callback function that reads the received joint position command and controls the real robot's joints accordingly.

## 6. Expected Output

The robot should smoothly follow the command with:
- **No vibration**
- **No overshoot**
- **Minimal delay**

By following these steps, you can ensure that your joint position controller meets the requirements of TeleXperience. Happy coding!


