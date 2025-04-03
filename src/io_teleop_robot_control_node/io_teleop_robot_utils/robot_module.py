import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__)))

import pybullet as p
import numpy as np
from dataclasses import dataclass
import time

from utils import multiply_transforms, invert_transform


class RobotDorsal(object):
    def __init__(
        self,
        robot,
        dorsal_lift_joint_index,
        home_j_pos,
        client_id=0,
    ):
        self.physics_clent_id = client_id
        self.robot = robot
        self.dorsal_lift_joint_index = dorsal_lift_joint_index
        self.home_j_pos = home_j_pos

    @property
    def height(self):
        return p.getJointState(self.robot.robot_id, self.dorsal_lift_joint_index)[0]

    def move(self, height):
        self.robot.move_j([self.dorsal_lift_joint_index], [height])


class RobotHead(object):
    def __init__(
        self,
        robot,
        head_joint_index,
        rest_j_pos,
        client_id=0,
    ):
        self.physics_clent_id = client_id
        self.robot = robot
        self.head_joint_index = head_joint_index  # in order: [roll, pitch, yaw]
        self.home_j_pos = rest_j_pos

    def move(self, joint):
        for i, joint_id in enumerate(self.head_joint_index):
            if joint_id != -1:
                self.robot.move_j([joint_id], [joint[i]])

    @property
    def rotation(self):
        rot = []
        for i, joint_id in enumerate(self.head_joint_index):
            if joint_id != -1:
                rot.append(p.getJointState(self.robot.robot_id, joint_id)[0])
            else:
                rot.append(joint_id)
        return rot

    def reset(self):
        for i, joint_id in enumerate(self.head_joint_index):
            if joint_id != -1:
                self.robot.move_j([joint_id], [self.home_j_pos[i]])


class RobotBase(object):
    def __init__(
        self,
        robot,
        client_id=0,
    ):
        self.physics_clent_id = client_id
        self.robot = robot

        self.trans = multiply_transforms(
            invert_transform(p.getBasePositionAndOrientation(self.robot.robot_id)),
            p.getDynamicsInfo(self.robot.robot_id, -1)[3:5],
        )

    @property
    def pose(self):
        return multiply_transforms(
            p.getBasePositionAndOrientation(self.robot.robot_id), self.trans
        )

    def reset_base(self, pose):
        pose = multiply_transforms(pose, invert_transform(self.trans))
        p.resetBasePositionAndOrientation(
            self.robot.robot_id, pose[0], pose[1], physicsClientId=self.physics_clent_id
        )


class RobotArm(object):
    def __init__(
        self,
        robot,
        arm_joint_index,
        ee_index,
        home_j_pos,
        with_ee_constraint,
        client_id=0,
    ):
        self.physics_clent_id = client_id
        self.robot = robot
        self.arm_joint_index = arm_joint_index
        self.ee_index = ee_index
        self.home_j_pos = home_j_pos

        self.robot.reset_j(self.arm_joint_index, self.home_j_pos)
        self.home_ee_pose = self.rel_ee_pose
        self.with_ee_constraint = with_ee_constraint
        if self.with_ee_constraint:
            # add ee constraint for robot arm movement
            visual_shape_id = p.createVisualShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[0.0001, 0.0001, 0.0002],
                rgbaColor=[1, 0, 0, 1],
            )
            self.box_id = p.createMultiBody(
                baseMass=0.1,
                baseInertialFramePosition=[0, 0, 0],
                baseVisualShapeIndex=visual_shape_id,
                basePosition=p.getLinkState(self.robot.robot_id, self.ee_index)[0],
                baseOrientation=p.getLinkState(self.robot.robot_id, self.ee_index)[1],
            )

            # fix box with robot base, change the position of box to change the ee pose
            self.box_base_cons = p.createConstraint(
                self.box_id,
                -1,
                self.robot.robot_id,
                -1,
                p.JOINT_FIXED,
                [0, 0, 0],
                [0, 0, 0],
                self.rel_ee_pose[0],
                [0, 0, 0, 1],
                self.rel_ee_pose[1],
            )
            # fix box with ee
            self.box_ee_cons = p.createConstraint(
                self.box_id,
                -1,
                self.robot.robot_id,
                self.ee_index,
                p.JOINT_FIXED,
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
            )

    @property
    def joint_pos(self):
        return [
            p.getJointState(self.robot.robot_id, i)[0] for i in self.arm_joint_index
        ]

    @property
    def abs_ee_pose(self):
        ee_pose = p.getLinkState(
            self.robot.robot_id, self.ee_index, physicsClientId=self.physics_clent_id
        )[4:6]
        return ee_pose

    @property
    def rel_ee_pose(self):
        # ee pose relative to robot base pose
        ee_pose = p.getLinkState(
            self.robot.robot_id, self.ee_index, physicsClientId=self.physics_clent_id
        )[4:6]
        base_pose = self.robot.base.pose
        inv_base_pose = p.invertTransform(base_pose[0], base_pose[1])
        ee_in_base_frame = multiply_transforms(inv_base_pose, ee_pose)
        return ee_in_base_frame

    def move_ee(self, ee_pose):
        # ee pose is relative to world frame, aka, move_abs_ee
        if self.with_ee_constraint:
            # convert from abs_ee to rel_ee
            inv_base_pose = invert_transform(self.robot.base.pose)
            ee_pose = multiply_transforms(inv_base_pose, ee_pose)
            self.robot.move_j(
                self.arm_joint_index, self.home_j_pos
            )  # to remain near home j pos
            inv_init_ee_pose = p.invertTransform(
                self.home_ee_pose[0], self.home_ee_pose[1]
            )
            delta_ee_pose = multiply_transforms(inv_init_ee_pose, ee_pose)
            inv_init_base_pose = p.invertTransform(
                self.robot.configs["base_pose"]["position"],
                self.robot.configs["base_pose"]["orientation"],
            )
            delta_base_pose = multiply_transforms(
                inv_init_base_pose, self.robot.base.pose
            )
            quat = delta_ee_pose[1]
            delta_ee_pose = list(delta_ee_pose)
            euler = -np.array(p.getEulerFromQuaternion(quat))
            delta_ee_pose[1] = p.getQuaternionFromEuler(euler)
            delta_pose = multiply_transforms(delta_ee_pose, delta_base_pose)

            p.changeConstraint(self.box_base_cons, ee_pose[0])
            p.changeConstraint(self.box_ee_cons, [0, 0, 0], delta_pose[1])


class RobotGripper(object):
    def __init__(
        self,
        robot,
        finger_index,
        client_id=0,
    ):
        self.physics_clent_id = client_id
        self.robot = robot
        self.finger_index = finger_index

        self.joint_range = [
            p.getJointInfo(
                self.robot.robot_id,
                i,
                physicsClientId=self.physics_clent_id,
            )[8:10]
            for i in self.finger_index
        ]

    @property
    def joint_pos(self):
        return [p.getJointState(self.robot.robot_id, i)[0] for i in self.finger_index]

    def reset(self, gripper_status):
        # assume gripper_status is a float in range [0, 1], 0 for open, 1 for close
        pos = [
            gripper_status * (self.joint_range[i][1] - self.joint_range[i][0])
            + self.joint_range[i][0]
            for i, index in enumerate(self.finger_index)
        ]

        self.robot.reset_j(self.finger_index, pos)

    def move(self, gripper_status):
        # assume gripper_status is a float in range [0, 1], 0 for open, 1 for close
        pos = [
            gripper_status * (self.joint_range[i][1] - self.joint_range[i][0])
            + self.joint_range[i][0]
            for i, index in enumerate(self.finger_index)
        ]
        self.robot.move_j(self.finger_index, pos)

    @property
    def status(self):
        # return gripper status in range [0, 1]
        sum_min = sum([min for (min, max) in self.joint_range])
        sum_max = sum([max for (min, max) in self.joint_range])
        sum_cur = sum(self.joint_pos)
        return np.clip((sum_cur - sum_min) / (sum_max - sum_min), 0, 1)

    def joint_pos_from_status(self, status):
        joint_pos = []
        for i, (min, max) in enumerate(self.joint_range):
            joint_pos.append(min + (max - min) * status)
        return joint_pos


@dataclass
class RobotCamera:
    robot: object
    camera_name: str
    camera_quat: tuple
    camera_height: float
    pos_offset: tuple
    target_offset: tuple
    near_val: float
    far_val: float
    up_axis: tuple
    img_width: int
    img_height: int
    physics_clent_id: int = 0

    def get_cam_base_pose(self):
        if self.camera_name == "first_person":
            return self.robot.base.pose
        elif "hand_eye" in self.camera_name:
            arm_id = 0 if "right" in self.camera_name else 1
            return self.robot.arms[arm_id].abs_ee_pose
        else:
            raise ValueError("Unsupported camera name")

    def get_img(self):
        eye_pos, eye_ori = multiply_transforms(
            self.get_cam_base_pose(),
            [[0, 0, 0], self.camera_quat],
        )
        r_mat = p.getMatrixFromQuaternion(eye_ori)
        ty_vec = np.array([r_mat[1], r_mat[4], r_mat[7]])
        tz_vec = np.array([r_mat[2], r_mat[5], r_mat[8]])
        camera_position = (
            np.array(eye_pos)
            + np.array([0, 0, self.camera_height])
            + np.array(self.pos_offset) * tz_vec
        )
        target_position = eye_pos + np.array(self.target_offset) * tz_vec
        cameraUpMultiplier = np.array(self.up_axis)

        cam_view_matrix = p.computeViewMatrix(
            cameraEyePosition=camera_position,
            cameraTargetPosition=target_position,
            cameraUpVector=cameraUpMultiplier * ty_vec,  # up vector in image
        )
        cam_projection_matrix = p.computeProjectionMatrixFOV(
            fov=100,
            aspect=float(self.img_width / self.img_height),
            nearVal=self.near_val,
            farVal=self.far_val,
        )
        _, _, image, _, _ = p.getCameraImage(
            self.img_width,
            self.img_height,
            cam_view_matrix,
            cam_projection_matrix,
            shadow=1,
            renderer=p.ER_BULLET_HARDWARE_OPENGL,
            flags=p.ER_NO_SEGMENTATION_MASK,
            physicsClientId=self.physics_clent_id,
        )
        return image


class AssembledRobot:
    def __init__(
        self, robot_urdf_path, configs, debug, with_ee_constraint, physics_clent_id
    ):
        self.load_urdf(robot_urdf_path, configs, debug, physics_clent_id)
        self.assemble_robot(configs, with_ee_constraint, physics_clent_id)

    def load_urdf(self, robot_urdf_path, configs, debug, physics_clent_id):
        use_fixed_base = not "fixed_link" in configs
        self.robot_id = p.loadURDF(
            fileName=robot_urdf_path,
            basePosition=configs["base_pose"]["position"],
            baseOrientation=configs["base_pose"]["orientation"],
            useFixedBase=use_fixed_base,
            flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES,
            physicsClientId=physics_clent_id,
        )

        self.joint_name2id_dict = {}
        self.joint_id2name_dict = {}
        self.link_name2id_dict = {}

        for i in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode("utf-8")
            link_name = joint_info[12].decode("utf-8")
            joint_id = joint_info[0]
            self.joint_id2name_dict[joint_id] = joint_name
            self.joint_name2id_dict[joint_name] = joint_id
            self.link_name2id_dict[link_name] = joint_id
        if not use_fixed_base:
            # set fixed base link
            constraint_id = p.createConstraint(
                parentBodyUniqueId=self.robot_id,
                parentLinkIndex=self.link_name2id_dict[
                    configs["fixed_link"]
                ],  # 固定的link ID
                childBodyUniqueId=-1,  # 与世界坐标系连接
                childLinkIndex=-1,  # 固定到世界
                jointType=p.JOINT_FIXED,  # 固定连接
                jointAxis=[0, 0, 0],
                parentFramePosition=[0, 0, 0],  # 父link的坐标
                childFramePosition=configs["base_pose"]["position"],
                childFrameOrientation=configs["base_pose"]["orientation"],
            )

        if debug:
            # debug usage, add joint state debug bars
            joint_index_debug_param_dt = {}
            for i in range(0, p.getNumJoints(self.robot_id)):
                joint_info = p.getJointInfo(self.robot_id, i)
                joint_type = None
                if joint_info[2] == p.JOINT_REVOLUTE:
                    joint_type = "REVOLUTE"
                elif joint_info[2] == p.JOINT_PRISMATIC:
                    joint_type = "PRISMATIC"
                elif joint_info[2] == p.JOINT_SPHERICAL:
                    joint_type = "SPHERICAL"
                elif joint_info[2] == p.JOINT_FIXED:
                    joint_type = "FIXED"
                print(
                    f"joint&parent_link_id: {str(joint_info[0]).ljust(5)} joint_type:{joint_type.ljust(10)} joint_name: {joint_info[1].decode('utf-8').ljust(28)} parent_link_name: {joint_info[12].decode('utf-8').ljust(28)}"
                )
                if joint_info[2] != p.JOINT_FIXED:
                    joint_index_debug_param_dt[i] = p.addUserDebugParameter(
                        paramName=f"   {i}    {joint_info[1].decode('utf-8')}",
                        rangeMin=joint_info[8],
                        rangeMax=joint_info[9],
                        startValue=p.getJointState(self.robot_id, i)[0],
                    )
            while True:
                for joint_index, param_id in joint_index_debug_param_dt.items():
                    p.resetJointState(
                        self.robot_id, joint_index, p.readUserDebugParameter(param_id)
                    )
                time.sleep(0.1)
        return self.robot_id

    def assemble_robot(self, configs, with_ee_constraint, physics_clent_id):
        self.base = RobotBase(self, physics_clent_id)
        self.arms = []
        for arm_config in configs["arms"]:
            arm = RobotArm(
                self,
                arm_config["joint_index"],
                arm_config["ee_index"],
                arm_config["rest_j_pos"],
                with_ee_constraint,
                self.physics_clent_id,
            )
            self.arms.append(arm)
        if "grippers" in configs:
            self.grippers = []
            for gripper_config in configs["grippers"]:
                gripper = RobotGripper(
                    self,
                    gripper_config["joint_index"],
                    client_id=physics_clent_id,
                )
                self.grippers.append(gripper)
                # adjust the mass and friction of gripper fingers
                for i in gripper.finger_index:
                    p.changeDynamics(
                        self.robot_id,
                        i,
                        mass=0.1,
                        lateralFriction=1000,
                        physicsClientId=physics_clent_id,
                    )
                if len(gripper.finger_index) == 2:
                    # for two-finger gripper, add a constraint to keep the two fingers symmetric
                    c = p.createConstraint(
                        self.robot_id,
                        gripper.finger_index[0],
                        self.robot_id,
                        gripper.finger_index[1],
                        jointType=p.JOINT_GEAR,
                        jointAxis=[1, 0, 0],
                        parentFramePosition=[0, 0, 0],
                        childFramePosition=[0, 0, 0],
                    )
                    p.changeConstraint(c, gearRatio=-1, erp=0.1, maxForce=50)
        if "cameras" in self.configs:
            self.cameras = []
            for cam_config in self.configs["cameras"]:
                cam = RobotCamera(
                    self,
                    cam_config["name"],
                    cam_config["quat"],
                    cam_config["height"],
                    cam_config["pos_offset"],
                    cam_config["target_offset"],
                    cam_config["near_val"],
                    cam_config["far_val"],
                    cam_config["up_axis"],
                    cam_config["img_width"],
                    cam_config["img_height"],
                    self.physics_clent_id,
                )
                self.cameras.append(cam)
        if "dorsal" in self.configs:
            self.dorsal = RobotDorsal(
                self,
                self.configs["dorsal"]["joint_index"],
                self.configs["dorsal"].get("rest_j_pos", 0.0),
                self.physics_clent_id,
            )

        if "head" in self.configs:
            self.head = RobotHead(
                self,
                self.configs["head"]["joint_index"],
                self.configs["head"]["rest_j_pos"],
                self.physics_clent_id,
            )

    def reset_j(self, j_index, j_pos):
        p.resetJointStatesMultiDof(
            self.robot_id,
            j_index,
            [[j] for j in j_pos],
            physicsClientId=self.physics_clent_id,
        )

    def move_j(self, j_index, j_pos, max_vel=[], force=[]):
        # give super large max_vel and force to ensure move speed and force
        # max_vel = [10] * len(j_index)
        force = [100] * len(j_index)
        if not max_vel or not force:
            max_vel = force = []
            for j_idx in j_index:
                joint_info = p.getJointInfo(self.robot_id, j_idx)
                max_vel.append(joint_info[11])
                force.append(joint_info[10])
        for i, j_idx in enumerate(j_index):
            p.setJointMotorControl2(
                self.robot_id,
                j_idx,
                p.POSITION_CONTROL,
                j_pos[i],
                maxVelocity=p.getJointInfo(self.robot_id, j_idx)[11],
                force=p.getJointInfo(self.robot_id, j_idx)[10],
                physicsClientId=self.physics_clent_id,
            )
