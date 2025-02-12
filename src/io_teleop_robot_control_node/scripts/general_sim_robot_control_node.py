import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(
    os.path.join(os.path.dirname(__file__), "mmk2_demo_sdk_client/mmk2_demo/mmk2_grpc")
)

import time
import threading
import yaml
import numpy as np
import pybullet as p
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger, TriggerResponse
import argparse

from io_teleop_robot_utils.robot_module import AssembledRobot

from io_teleop_robot_utils.utils import (
    debug_draw_pose,
    multiply_transforms,
    pose_msg_to_list,
)


class SimRobotController(AssembledRobot):
    def __init__(self, config_path, init_debug=False):
        client_id = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraPitch=-30,
            cameraYaw=180,
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
        debug_draw_pose(self.base.pose)
        self.ids = [None, None]
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
        if "fixed_link" not in self.configs:
            self.target_base_move_sub = rospy.Subscriber(
                f"/io_teleop/target_base_move",
                Float64MultiArray,
                self.target_base_move_callback,
            )
        self.target_ee_sub = rospy.Subscriber(
            f"/io_teleop/target_ee_poses",
            PoseArray,
            self.target_ee_callback,
        )
        self.joint_state_pub_rate = 100
        rospy.Timer(rospy.Duration(1.0 / 100), self.update_joint_state)
        self.reset_service_server = rospy.Service(
            "io_teleop_reset_robot", Trigger, self.reset_service_callback
        )

    def reset_service_callback(self, req):
        print(
            "===========================Reset robot to home request received==========================="
        )
        self.reset_home()
        time.sleep(1)
        response = TriggerResponse()
        response.success = True
        response.message = "Service successfully triggered!"
        return response

    def update_joint_state(self, event):
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time().now()
        joint_state.name = []
        joint_state.position = []
        for i in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, i)
            if joint_info[2] == p.JOINT_FIXED:
                continue
            joint_state.name.append(joint_info[1].decode("utf-8"))
            joint_state.position.append(p.getJointState(self.robot_id, i)[0])
        self.joint_state_pub.publish(joint_state)

    def joint_cmd_callback(self, msg):
        joint_pos = msg.position
        joint_names = msg.name
        for pos, name in zip(joint_pos, joint_names):
            joint_id = self.joint_name2id_dict[name]
            self.reset_j([joint_id], [pos])

    def joint_cmd_from_vr_callback(self, msg):
        joint_names = msg.name
        joint_positions = msg.position
        for pos, name in zip(joint_positions, joint_names):
            joint_id = self.joint_name2id_dict[name]
            self.reset_j([joint_id], [pos])

    def gripper_status_callback(self, msg):
        gripper_status = msg.position
        for i, gripper in enumerate(self.grippers):
            gripper.reset(gripper_status[i])

    def target_ee_callback(self, msg):
        if "controller_indices" in self.configs:
            for i in range(len(self.arms)):
                pose = msg.poses[i]
                id = self.configs["controller_indices"]["base"][i]
                pose_list = pose_msg_to_list(pose)
                cmd_base_pose = (
                    p.getLinkState(self.robot_id, id)[4:6]
                    if id != -1
                    else self.base.pose
                )
                if self.ids[i] == None:
                    self.ids[i] = debug_draw_pose(
                        multiply_transforms(cmd_base_pose, pose_list)
                    )
                else:
                    debug_draw_pose(
                        multiply_transforms(cmd_base_pose, pose_list), self.ids[i]
                    )
        else:
            time.sleep(0.001)

    def target_base_move_callback(self, msg):
        delta_pos = msg.data[1]
        delta_yaw = msg.data[0]
        tar_pose = multiply_transforms(
            self.base.pose,
            [[0, delta_pos, 0], p.getQuaternionFromEuler([0, 0, delta_yaw])],
        )
        self.base.reset_base(tar_pose)

    def reset_home(self):
        for i, arm in enumerate(self.arms):
            self.reset_j(arm.arm_joint_index, arm.home_j_pos)

        if hasattr(self, "grippers"):
            for i, gripper in enumerate(self.grippers):
                gripper.reset(0)
        if hasattr(self, "head"):
            for i, pos in zip(self.head.head_joint_index, self.head.home_j_pos):
                if i != -1:
                    self.reset_j([i], [pos])

        if hasattr(self, "dorsal"):
            self.reset_j(
                [self.dorsal.dorsal_lift_joint_index], [self.dorsal.home_j_pos]
            )


if __name__ == "__main__":
    rospy.init_node("sim_robot_controller_node")
    args = argparse.ArgumentParser()
    args.add_argument("--robot_name", type=str)
    args = args.parse_args()
    config_path = args.robot_name + "/vr_configs.yml"
    robot = SimRobotController(config_path)
    rospy.spin()
