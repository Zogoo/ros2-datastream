/** Topic registry — the only place topic names and types live on the FE side.
 *  Mirrors src/onsen_robot_state/onsen_robot_state/topics.py. */
export const TOPICS = {
  // FE publishes (sensors / ground truth)
  scan: { name: '/scan', type: 'sensor_msgs/LaserScan' },
  camFrontCompressed: { name: '/camera/front/image_raw/compressed', type: 'sensor_msgs/CompressedImage' },
  camFrontInfo: { name: '/camera/front/camera_info', type: 'sensor_msgs/CameraInfo' },
  camRearCompressed: { name: '/camera/rear/image_raw/compressed', type: 'sensor_msgs/CompressedImage' },
  camRearInfo: { name: '/camera/rear/camera_info', type: 'sensor_msgs/CameraInfo' },
  camDepth: { name: '/camera/depth/image_raw', type: 'sensor_msgs/Image' },
  camDepthInfo: { name: '/camera/depth/camera_info', type: 'sensor_msgs/CameraInfo' },
  sonar: (i) => ({ name: `/sonar/range_${i}`, type: 'sensor_msgs/Range' }),
  imu: { name: '/imu', type: 'sensor_msgs/Imu' },
  odom: { name: '/odom', type: 'nav_msgs/Odometry' },
  tf: { name: '/tf', type: 'tf2_msgs/TFMessage' },
  jointStates: { name: '/joint_states', type: 'sensor_msgs/JointState' },
  contacts: { name: '/robot/contacts', type: 'std_msgs/String' },
  events: { name: '/robot/events', type: 'std_msgs/String' },
  groundTruthPose: { name: '/ground_truth/pose', type: 'geometry_msgs/PoseStamped' },
  groundTruthObjects: { name: '/ground_truth/objects', type: 'std_msgs/String' },
  simStatus: { name: '/sim/status', type: 'std_msgs/String' },

  // FE publishes (control)
  cmdVelUi: { name: '/cmd_vel/ui', type: 'geometry_msgs/Twist' },
  controlModeSet: { name: '/robot/control_mode/set', type: 'std_msgs/String' },
  armCommand: { name: '/arm/command', type: 'std_msgs/String' },
  baseCommand: { name: '/base/command', type: 'std_msgs/String' },

  // FE subscribes
  armJointTargets: { name: '/arm/joint_targets', type: 'std_msgs/String' },
  baseWheelTargets: { name: '/base/wheel_targets', type: 'std_msgs/String' },
  safetyStop: { name: '/safety/stop', type: 'std_msgs/Bool' },
  armResponse: { name: '/arm/response', type: 'std_msgs/String' },
  baseResponse: { name: '/base/response', type: 'std_msgs/String' },
  controlMode: { name: '/robot/control_mode', type: 'std_msgs/String' },
  detectedObjects: { name: '/detected_objects', type: 'std_msgs/String' },
  taskPlan: { name: '/task_plan', type: 'std_msgs/String' },
  missionState: { name: '/mission/state', type: 'std_msgs/String' },
};
