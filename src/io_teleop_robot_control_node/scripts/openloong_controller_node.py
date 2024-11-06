import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
import time
import threading
import yaml
import numpy as np
import pybullet as p
import rospy
from sensor_msgs.msg import JointState
from io_teleop_robot_utils.robot_module import AssembledRobot


class SimOpenLoongController(AssembledRobot):
    def __init__(self, config_path, init_debug=False):
        client_id = p.connect(p.DIRECT)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
        p.resetDebugVisualizerCamera(
            cameraDistance=1,
            cameraPitch=-60,
            cameraYaw=0,
            cameraTargetPosition=[0, 0, 0.6],
        )

        # start simulation
        def physics_sim():
            # p.setRealTimeSimulation(1)
            while not rospy.is_shutdown():
                p.stepSimulation()
                time.sleep(1.0 / 240.0)

        physics_sim_thread = threading.Thread(target=physics_sim)
        physics_sim_thread.start()
        self.physics_clent_id = client_id
        self.robot_description_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "../io_teleop_robot_descriptions",
        )
        self.configs = yaml.safe_load(
            open(f"{self.robot_description_path}/{config_path}")
        )
        self.wo_controller = False
        robot_urdf_path = f"{self.robot_description_path}/{os.path.dirname(config_path)}/{self.configs['urdf_path']}"
        with_ee_constraint = False
        AssembledRobot.__init__(
            self,
            robot_urdf_path,
            self.configs,
            init_debug,
            with_ee_constraint,
            self.physics_clent_id,
        )
        self.reset_home()
        # get joint state and then pub to controller
        self.joint_state_pub = rospy.Publisher(
            f"/io_teleop/joint_states", JointState, queue_size=1
        )
        self.joint_cmd_sub = rospy.Subscriber(
            f"/io_teleop/joint_cmd",
            JointState,
            self.joint_cmd_callback,
        )
        self.joint_cmd_from_vr_sub = rospy.Subscriber(
            "/io_teleop/target_joint_from_vr",
            JointState,
            self.joint_cmd_from_vr_callback,
        )
        self.gripper_status_sub = rospy.Subscriber(
            "/io_teleop/target_gripper_status",
            JointState,
            self.gripper_status_callback,
        )
        self.joint_state_pub_rate = rospy.Rate(100)
        joint_state_pub_thread = threading.Thread(target=self.joint_state_pub_thread)
        joint_state_pub_thread.Daemon = True
        joint_state_pub_thread.start()

    def joint_state_pub_thread(self):
        joint_state = JointState()
        while not rospy.is_shutdown():
            joint_state.header.stamp = rospy.Time.now()
            joint_state.name = []
            joint_state.position = []
            for i in range(p.getNumJoints(self.robot_id)):
                joint_info = p.getJointInfo(self.robot_id, i)
                if joint_info[2] == p.JOINT_FIXED:
                    continue
                joint_state.name.append(joint_info[1].decode("utf-8"))
                joint_state.position.append(p.getJointState(self.robot_id, i)[0])
            self.joint_state_pub.publish(joint_state)
            self.joint_state_pub_rate.sleep()

    def joint_cmd_callback(self, msg):
        joint_pos = msg.position
        joint_names = msg.name
        for pos, name in zip(joint_pos, joint_names):
            joint_id = self.joint_name2id_dict[name]
            self.move_j([joint_id], [pos])

    def joint_cmd_from_vr_callback(self, msg):
        joint_names = msg.name
        joint_positions = msg.position
        for pos, name in zip(joint_positions, joint_names):
            joint_id = self.joint_name2id_dict[name]
            self.move_j([joint_id], [pos])

    def reset_home(self):
        for i, arm in enumerate(self.arms):
            self.move_j(arm.arm_joint_index, arm.home_j_pos)
        waist_config = self.configs["waist"]
        self.move_j(waist_config["joint_index"], waist_config["rest_j_pos"])

        if hasattr(self, "grippers"):
            for i, gripper in enumerate(self.grippers):
                gripper.reset(0)

    def gripper_status_callback(self, msg):
        gripper_status = msg.position
        for i, gripper in enumerate(self.grippers):
            gripper.move(gripper_status[i])


if __name__ == "__main__":
    rospy.init_node("openloong_controller_node")
    robot = SimOpenLoongController("OpenLoong/vr_configs.yml")
    rospy.spin()
