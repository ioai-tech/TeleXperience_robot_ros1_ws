# io_teleop_robot_control_node

## DEPENDIENCES

ROS1 required

```bash
pip3 install -r requirements.txt
```

## RUN

### for sim constraint robot *wo* any controller:


e.g.:
```bash
python3 robots/uni_constraint_control_node.py --robot_name RM75
```

If use uni_constraint_control_node, io_unicontroller is not needed

### for sim OpenLoong robot *with* io_unicontroller:
```bash
python3 robots/openloong_controller_node.py
```
```