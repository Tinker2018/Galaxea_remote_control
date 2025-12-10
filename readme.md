nvidia@nvidia-desktop:~/galaxea/install/startup_config/share/startup_config/script$ ros2 node info /vr_controller 
/vr_controller
  Subscribers:
    <!-- /hdas/feedback_arm_left: sensor_msgs/msg/JointState
    /hdas/feedback_arm_right: sensor_msgs/msg/JointState
    /hdas/feedback_gripper_left: sensor_msgs/msg/JointState
    /hdas/feedback_gripper_right: sensor_msgs/msg/JointState
    /motion_control/pose_ee_arm_left: geometry_msgs/msg/PoseStamped
    /motion_control/pose_ee_arm_right: geometry_msgs/msg/PoseStamped
    /motion_target/target_pose_arm_left: geometry_msgs/msg/PoseStamped
    /motion_target/target_pose_arm_right: geometry_msgs/msg/PoseStamped -->
    <!-- /parameter_events: rcl_interfaces/msg/ParameterEvent
    /vr_pose: teleoperation_msg_ros2/msg/VrPose -->
  Publishers:
    /motion_control/control_arm_left: hdas_msg/msg/MotorControl
    /motion_control/control_arm_right: hdas_msg/msg/MotorControl
    <!-- /motion_target/target_joint_state_arm_left: sensor_msgs/msg/JointState
    /motion_target/target_joint_state_arm_right: sensor_msgs/msg/JointState -->
    <!-- /motion_target/target_pose_arm_left: geometry_msgs/msg/PoseStamped
    /motion_target/target_pose_arm_right: geometry_msgs/msg/PoseStamped -->
    <!-- /motion_target/target_position_gripper_left: sensor_msgs/msg/JointState
    /motion_target/target_position_gripper_right: sensor_msgs/msg/JointState -->
    /motion_target/target_speed_chassis: geometry_msgs/msg/TwistStamped
    /motion_target/target_speed_torso: geometry_msgs/msg/TwistStamped
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /switch_control_mode_vr: teleoperation_msg_ros2/srv/SwitchControlModeVR
    /vr_controller/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /vr_controller/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /vr_controller/get_parameters: rcl_interfaces/srv/GetParameters
    /vr_controller/list_parameters: rcl_interfaces/srv/ListParameters
    /vr_controller/set_parameters: rcl_interfaces/srv/SetParameters
    /vr_controller/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:
    /start_data_collection: std_srvs/srv/Trigger
    /stop_data_collection: std_srvs/srv/Trigger
    /system_manager/teleop/service: system_manager_msg/srv/TeleopFrame
  Action Servers:

  Action Clients:

nvidia@nvidia-desktop:~/galaxea/install/startup_config/share/startup_config/script$ ros2 node info /vr_data_receiver 
/vr_data_receiver
  <!-- Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent -->
    /rosout: rcl_interfaces/msg/Log
    /vr_pose: teleoperation_msg_ros2/msg/VrPose
  Service Servers:
    /vr_data_receiver/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /vr_data_receiver/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /vr_data_receiver/get_parameters: rcl_interfaces/srv/GetParameters
    /vr_data_receiver/list_parameters: rcl_interfaces/srv/ListParameters
    /vr_data_receiver/set_parameters: rcl_interfaces/srv/SetParameters
    /vr_data_receiver/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:


scp -rp gala* nvidia@10.0.42.184:~/tinker_ws





