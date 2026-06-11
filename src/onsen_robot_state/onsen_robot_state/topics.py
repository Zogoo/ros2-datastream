"""Topic contract — the only place topic names live on the Python side.

Mirrored by frontend/src/ros/topics.js. Every node imports from here so the
contract in docs/topics.md can never drift from the code.
"""

# Sensors / ground truth (FE simulator -> ROS)
SCAN = "/scan"
CAM_FRONT_COMPRESSED = "/camera/front/image_raw/compressed"
CAM_FRONT_INFO = "/camera/front/camera_info"
CAM_REAR_COMPRESSED = "/camera/rear/image_raw/compressed"
CAM_REAR_INFO = "/camera/rear/camera_info"
CAM_DEPTH = "/camera/depth/image_raw"
CAM_DEPTH_INFO = "/camera/depth/camera_info"
IMU = "/imu"
ODOM = "/odom"
JOINT_STATES = "/joint_states"
CONTACTS = "/robot/contacts"
EVENTS = "/robot/events"
GROUND_TRUTH_POSE = "/ground_truth/pose"
GROUND_TRUTH_OBJECTS = "/ground_truth/objects"
SIM_STATUS = "/sim/status"


def sonar(index: int) -> str:
    return f"/sonar/range_{index}"


# Control path
CMD_VEL = "/cmd_vel"
CMD_VEL_UI = "/cmd_vel/ui"
CMD_VEL_AUTO = "/cmd_vel/auto"
CONTROL_MODE = "/robot/control_mode"
CONTROL_MODE_SET = "/robot/control_mode/set"

# Arm firmware (serial protocol over topics)
ARM_COMMAND = "/arm/command"
ARM_RESPONSE = "/arm/response"
ARM_JOINT_TARGETS = "/arm/joint_targets"
ARM_STATE = "/arm/state"

# Base firmware (wheel protocol over topics)
BASE_COMMAND = "/base/command"
BASE_RESPONSE = "/base/response"
BASE_WHEEL_TARGETS = "/base/wheel_targets"
BASE_STATE = "/base/state"

# Safety / fused state (this package)
SAFETY_STOP = "/safety/stop"
SAFETY_RESET = "/safety/reset"
SAFETY_ENABLE = "/safety/enable"
ROBOT_STATE = "/robot/state"

# Perception / autonomy
DETECTED_OBJECTS = "/detected_objects"
TASK_PLAN = "/task_plan"
MISSION_STATE = "/mission/state"
