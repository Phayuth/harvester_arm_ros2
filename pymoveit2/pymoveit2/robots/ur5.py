from typing import List

MOVE_GROUP_ARM: str = "ur_manipulator"
MOVE_GROUP_GRIPPER: str = "gripper"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.04, 0.04]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0, 0.0]


def joint_names() -> List[str]:
    return [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]


def base_link_name() -> str:
    return "base_link"


def end_effector_name() -> str:
    return "tool0"


# def gripper_joint_names() -> List[str]:
#     return [
#         prefix + "leftfinger",
#         prefix + "rightfinger",
#     ]