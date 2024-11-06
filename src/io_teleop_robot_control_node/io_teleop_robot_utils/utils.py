import pybullet as p
import numpy as np


def debug_draw_pose(pose, replace_line_ids=None, line_width=5, line_length=0.3):
    axes = np.identity(3) * line_length
    colors = np.identity(3)
    line_ids = [] if replace_line_ids is None else replace_line_ids
    for i in range(3):
        if replace_line_ids is None:
            line_id = p.addUserDebugLine(
                lineFromXYZ=pose[0],
                lineToXYZ=p.multiplyTransforms(pose[0], pose[1], axes[i], [0, 0, 0, 1])[
                    0
                ],
                lineColorRGB=colors[i],
                lineWidth=line_width,
                lifeTime=0,
            )
            line_ids.append(line_id)
        else:
            p.addUserDebugLine(
                lineFromXYZ=pose[0],
                lineToXYZ=p.multiplyTransforms(pose[0], pose[1], axes[i], [0, 0, 0, 1])[
                    0
                ],
                lineColorRGB=colors[i],
                lineWidth=line_width,
                lifeTime=0,
                replaceItemUniqueId=replace_line_ids[i],
            )
    return line_ids


def multiply_transforms(pose1, pose2):
    return p.multiplyTransforms(pose1[0], pose1[1], pose2[0], pose2[1])


def invert_transform(pose):
    return p.invertTransform(pose[0], pose[1])


def pose_list_to_msg(pose):
    pose_msg = Pose()
    pose_msg.position.x = pose[0][0]
    pose_msg.position.y = pose[0][1]
    pose_msg.position.z = pose[0][2]
    pose_msg.orientation.x = pose[1][0]
    pose_msg.orientation.y = pose[1][1]
    pose_msg.orientation.z = pose[1][2]
    pose_msg.orientation.w = pose[1][3]

    return pose_msg


def pose_msg_to_list(msg):
    return [
        [msg.position.x, msg.position.y, msg.position.z],
        [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ],
    ]
