import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
import yaml
import pybullet as p
import time
import threading
import rospy
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import argparse

from io_teleop_robot_utils.robot_module import AssembledRobot

from io_teleop_robot_utils.utils import pose_msg_to_list, debug_draw_pose


class UniSimConstraintController(AssembledRobot):
    def __init__(self, config_path, init_debug=False):
        client_id = p.connect(p.GUI)
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
        self.wo_controller = True
        robot_urdf_path = f"{self.robot_description_path}/{os.path.dirname(config_path)}/{self.configs['urdf_path']}"
        with_ee_constraint = True
        AssembledRobot.__init__(
            self,
            robot_urdf_path,
            self.configs,
            init_debug,
            with_ee_constraint,
            self.physics_clent_id,
        )

        # get joint state and then pub to controller
        self.joint_state_pub = rospy.Publisher(
            f"/io_teleop/joint_states", JointState, queue_size=1
        )
        # sub cmd from retargeting module
        self.ee_cmd_sub = rospy.Subscriber(
            "/io_teleop/target_ee_poses", PoseArray, self.ee_cmd_callback
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
        self.base_movement_sub = rospy.Subscriber(
            "/io_teleop/target_base_move",
            Float64MultiArray,
            self.base_movement_callback,
        )
        self.joint_state_pub_rate = rospy.Rate(100)
        joint_state_pub_thread = threading.Thread(target=self.joint_state_pub_thread)
        joint_state_pub_thread.Daemon = True
        joint_state_pub_thread.start()

        self.robot_gripper_collision_status_pub = rospy.Publisher(
            f"/io_teleop/gripper_collision_status", JointState, queue_size=1
        )
        if hasattr(self, "grippers"):
            self.gripper_collision_status_pub_rate = rospy.Rate(10)
            gripper_collision_status_pub_thread = threading.Thread(
                target=self.gripper_collision_status_pub_thread
            )
            gripper_collision_status_pub_thread.Daemon = True
            gripper_collision_status_pub_thread.start()

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

    def gripper_collision_status_pub_thread(self):
        # check if hands collide with any objects
        gripper_collision_status = JointState(position=[0, 0])
        for i, gripper in enumerate(self.grippers):
            for j, idx in enumerate(gripper.finger_index):
                contact_info = p.getContactPoints(
                    bodyA=self.robot_id, linkIndexA=idx, physicsClientId=0
                )
                if len(contact_info) != 0:
                    gripper_collision_status[i] = 1.0
                    break
        self.robot_gripper_collision_status_pub.publish(gripper_collision_status)

    def ee_cmd_callback(self, msg):
        ee_poses = msg.poses
        ee_poses_list = [pose_msg_to_list(pose) for pose in ee_poses]
        try:
            [
                debug_draw_pose(ee_poses_list[i], self.ids[i])
                for i in range(len(ee_poses_list))
            ]
        except:
            self.ids = [debug_draw_pose(pose) for pose in ee_poses_list]
        for i, arm in enumerate(self.arms):
            arm.move_ee(ee_poses_list[i])

    def joint_cmd_from_vr_callback(self, msg):
        joint_names = msg.name
        joint_positions = msg.position
        for i, name in enumerate(joint_names):
            joint_id = self.joint_name2id_dict[name]
            self.move_j([joint_id], [joint_positions[i]])

    def gripper_status_callback(self, msg):
        gripper_status = msg.position
        for i, gripper in enumerate(self.grippers):
            gripper.move(gripper_status[i])

    def base_movement_callback(self, msg):
        base_motion = msg.data


if __name__ == "__main__":
    rospy.init_node("uni_controller_node")
    args = argparse.ArgumentParser()
    args.add_argument("--robot_name", type=str)
    args = args.parse_args()
    config_path = args.robot_name + "/vr_configs.yml"
    robot = UniSimConstraintController(config_path)
    debug_draw_pose(robot.arms[0].abs_ee_pose)
    debug_draw_pose(robot.arms[1].abs_ee_pose)
    rospy.spin()
