;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::CommandConstants)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'CommandConstants (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::COMMANDCONSTANTS")
  (make-package "FF_MSGS::COMMANDCONSTANTS"))

(in-package "ROS")
;;//! \htmlinclude CommandConstants.msg.html


(intern "*PARAM_NAME_LOCALIZATION_MODE_NONE*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_LOCALIZATION_MODE_NONE* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_LOCALIZATION_MODE_NONE* "None")
(intern "*PARAM_NAME_LOCALIZATION_MODE_MAPPED_LANDMARKS*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_LOCALIZATION_MODE_MAPPED_LANDMARKS* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_LOCALIZATION_MODE_MAPPED_LANDMARKS* "MappedLandmarks")
(intern "*PARAM_NAME_LOCALIZATION_MODE_ARTAGS*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_LOCALIZATION_MODE_ARTAGS* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_LOCALIZATION_MODE_ARTAGS* "ARTags")
(intern "*PARAM_NAME_LOCALIZATION_MODE_HANDRAIL*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_LOCALIZATION_MODE_HANDRAIL* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_LOCALIZATION_MODE_HANDRAIL* "Handrail")
(intern "*PARAM_NAME_LOCALIZATION_MODE_PERCH*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_LOCALIZATION_MODE_PERCH* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_LOCALIZATION_MODE_PERCH* "Perch")
(intern "*PARAM_NAME_LOCALIZATION_MODE_TRUTH*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_LOCALIZATION_MODE_TRUTH* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_LOCALIZATION_MODE_TRUTH* "Truth")
(intern "*PARAM_NAME_ACTION_TYPE_PAN*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_ACTION_TYPE_PAN* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_ACTION_TYPE_PAN* "Pan")
(intern "*PARAM_NAME_ACTION_TYPE_TILT*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_ACTION_TYPE_TILT* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_ACTION_TYPE_TILT* "Tilt")
(intern "*PARAM_NAME_ACTION_TYPE_BOTH*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_ACTION_TYPE_BOTH* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_ACTION_TYPE_BOTH* "Both")
(intern "*PARAM_NAME_POWERED_COMPONENT_LASER_POINTER*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_POWERED_COMPONENT_LASER_POINTER* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_POWERED_COMPONENT_LASER_POINTER* "Laser Pointer")
(intern "*PARAM_NAME_POWERED_COMPONENT_PAYLOAD_TOP_AFT*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_POWERED_COMPONENT_PAYLOAD_TOP_AFT* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_POWERED_COMPONENT_PAYLOAD_TOP_AFT* "Payload Top Aft")
(intern "*PARAM_NAME_POWERED_COMPONENT_PAYLOAD_BOTTOM_AFT*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_POWERED_COMPONENT_PAYLOAD_BOTTOM_AFT* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_POWERED_COMPONENT_PAYLOAD_BOTTOM_AFT* "Payload Bottom Aft")
(intern "*PARAM_NAME_POWERED_COMPONENT_PAYLOAD_BOTTOM_FRONT*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_POWERED_COMPONENT_PAYLOAD_BOTTOM_FRONT* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_POWERED_COMPONENT_PAYLOAD_BOTTOM_FRONT* "Payload Bottom Front")
(intern "*PARAM_NAME_POWERED_COMPONENT_PMCS_AND_SIGNAL_LIGHTS*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_POWERED_COMPONENT_PMCS_AND_SIGNAL_LIGHTS* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_POWERED_COMPONENT_PMCS_AND_SIGNAL_LIGHTS* "PMC")
(intern "*PARAM_NAME_CAMERA_MODE_BOTH*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_CAMERA_MODE_BOTH* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_CAMERA_MODE_BOTH* "Both")
(intern "*PARAM_NAME_CAMERA_MODE_RECORDING*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_CAMERA_MODE_RECORDING* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_CAMERA_MODE_RECORDING* "Recording")
(intern "*PARAM_NAME_CAMERA_MODE_STREAMING*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_CAMERA_MODE_STREAMING* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_CAMERA_MODE_STREAMING* "Streaming")
(intern "*PARAM_NAME_CAMERA_NAME_SCI*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_CAMERA_NAME_SCI* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_CAMERA_NAME_SCI* "Science")
(intern "*PARAM_NAME_CAMERA_NAME_NAV*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_CAMERA_NAME_NAV* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_CAMERA_NAME_NAV* "Navigation")
(intern "*PARAM_NAME_CAMERA_NAME_HAZ*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_CAMERA_NAME_HAZ* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_CAMERA_NAME_HAZ* "Hazard")
(intern "*PARAM_NAME_CAMERA_NAME_DOCK*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_CAMERA_NAME_DOCK* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_CAMERA_NAME_DOCK* "Dock")
(intern "*PARAM_NAME_CAMERA_NAME_PERCH*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_CAMERA_NAME_PERCH* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_CAMERA_NAME_PERCH* "Perch")
(intern "*PARAM_NAME_CAMERA_RESOLUTION_224X171*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_CAMERA_RESOLUTION_224X171* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_CAMERA_RESOLUTION_224X171* "224x171")
(intern "*PARAM_NAME_CAMERA_RESOLUTION_320X240*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_CAMERA_RESOLUTION_320X240* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_CAMERA_RESOLUTION_320X240* "320x240")
(intern "*PARAM_NAME_CAMERA_RESOLUTION_480X270*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_CAMERA_RESOLUTION_480X270* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_CAMERA_RESOLUTION_480X270* "480x270")
(intern "*PARAM_NAME_CAMERA_RESOLUTION_640X480*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_CAMERA_RESOLUTION_640X480* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_CAMERA_RESOLUTION_640X480* "640x480")
(intern "*PARAM_NAME_CAMERA_RESOLUTION_960X540*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_CAMERA_RESOLUTION_960X540* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_CAMERA_RESOLUTION_960X540* "960x540")
(intern "*PARAM_NAME_CAMERA_RESOLUTION_1024X768*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_CAMERA_RESOLUTION_1024X768* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_CAMERA_RESOLUTION_1024X768* "1024x768")
(intern "*PARAM_NAME_CAMERA_RESOLUTION_1280X720*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_CAMERA_RESOLUTION_1280X720* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_CAMERA_RESOLUTION_1280X720* "1280x720")
(intern "*PARAM_NAME_CAMERA_RESOLUTION_1280X960*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_CAMERA_RESOLUTION_1280X960* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_CAMERA_RESOLUTION_1280X960* "1280x960")
(intern "*PARAM_NAME_CAMERA_RESOLUTION_1920X1080*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_CAMERA_RESOLUTION_1920X1080* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_CAMERA_RESOLUTION_1920X1080* "1920x1080")
(intern "*PARAM_NAME_FLASHLIGHT_LOCATION_BACK*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_FLASHLIGHT_LOCATION_BACK* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_FLASHLIGHT_LOCATION_BACK* "Back")
(intern "*PARAM_NAME_FLASHLIGHT_LOCATION_FRONT*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_FLASHLIGHT_LOCATION_FRONT* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_FLASHLIGHT_LOCATION_FRONT* "Front")
(intern "*PARAM_NAME_FLIGHT_MODE_OFF*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_FLIGHT_MODE_OFF* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_FLIGHT_MODE_OFF* "off")
(intern "*PARAM_NAME_FLIGHT_MODE_QUIET*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_FLIGHT_MODE_QUIET* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_FLIGHT_MODE_QUIET* "quiet")
(intern "*PARAM_NAME_FLIGHT_MODE_NOMINAL*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_FLIGHT_MODE_NOMINAL* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_FLIGHT_MODE_NOMINAL* "nominal")
(intern "*PARAM_NAME_FLIGHT_MODE_DIFFICULT*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_FLIGHT_MODE_DIFFICULT* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_FLIGHT_MODE_DIFFICULT* "difficult")
(intern "*PARAM_NAME_FLIGHT_MODE_PRECISION*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_FLIGHT_MODE_PRECISION* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_FLIGHT_MODE_PRECISION* "precision")
(intern "*PARAM_NAME_PLANNER_TYPE_TRAPEZOIDAL*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_PLANNER_TYPE_TRAPEZOIDAL* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_PLANNER_TYPE_TRAPEZOIDAL* "trapezoidal")
(intern "*PARAM_NAME_PLANNER_TYPE_QUADRATIC_PROGRAM*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_PLANNER_TYPE_QUADRATIC_PROGRAM* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_PLANNER_TYPE_QUADRATIC_PROGRAM* "qp")
(intern "*PARAM_NAME_TELEMETRY_TYPE_COMM_STATUS*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_TELEMETRY_TYPE_COMM_STATUS* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_TELEMETRY_TYPE_COMM_STATUS* "CommStatus")
(intern "*PARAM_NAME_TELEMETRY_TYPE_CPU_STATE*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_TELEMETRY_TYPE_CPU_STATE* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_TELEMETRY_TYPE_CPU_STATE* "CpuState")
(intern "*PARAM_NAME_TELEMETRY_TYPE_DISK_STATE*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_TELEMETRY_TYPE_DISK_STATE* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_TELEMETRY_TYPE_DISK_STATE* "DiskState")
(intern "*PARAM_NAME_TELEMETRY_TYPE_EKF_STATE*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_TELEMETRY_TYPE_EKF_STATE* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_TELEMETRY_TYPE_EKF_STATE* "EkfState")
(intern "*PARAM_NAME_TELEMETRY_TYPE_GNC_STATE*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_TELEMETRY_TYPE_GNC_STATE* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_TELEMETRY_TYPE_GNC_STATE* "GncState")
(intern "*PARAM_NAME_TELEMETRY_TYPE_PMC_CMD_STATE*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_TELEMETRY_TYPE_PMC_CMD_STATE* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_TELEMETRY_TYPE_PMC_CMD_STATE* "PmcCmdState")
(intern "*PARAM_NAME_TELEMETRY_TYPE_POSITION*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_TELEMETRY_TYPE_POSITION* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_TELEMETRY_TYPE_POSITION* "Position")
(intern "*PARAM_NAME_TELEMETRY_TYPE_SPARSE_MAPPING_POSE*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*PARAM_NAME_TELEMETRY_TYPE_SPARSE_MAPPING_POSE* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*PARAM_NAME_TELEMETRY_TYPE_SPARSE_MAPPING_POSE* "SparseMappingPose")
(intern "*CMD_NAME_GRAB_CONTROL*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_GRAB_CONTROL* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_GRAB_CONTROL* "grabControl")
(intern "*CMD_NAME_REQUEST_CONTROL*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_REQUEST_CONTROL* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_REQUEST_CONTROL* "requestControl")
(intern "*CMD_NAME_FAULT*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_FAULT* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_FAULT* "fault")
(intern "*CMD_NAME_INITIALIZE_BIAS*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_INITIALIZE_BIAS* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_INITIALIZE_BIAS* "initializeBias")
(intern "*CMD_NAME_LOAD_NODELET*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_LOAD_NODELET* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_LOAD_NODELET* "loadNodelet")
(intern "*CMD_NAME_NO_OP*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_NO_OP* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_NO_OP* "noOp")
(intern "*CMD_NAME_REACQUIRE_POSITION*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_REACQUIRE_POSITION* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_REACQUIRE_POSITION* "reacquirePosition")
(intern "*CMD_NAME_RESET_EKF*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_RESET_EKF* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_RESET_EKF* "resetEkf")
(intern "*CMD_NAME_SWITCH_LOCALIZATION*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_SWITCH_LOCALIZATION* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_SWITCH_LOCALIZATION* "switchLocalization")
(intern "*CMD_NAME_UNLOAD_NODELET*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_UNLOAD_NODELET* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_UNLOAD_NODELET* "unloadNodelet")
(intern "*CMD_NAME_UNTERMINATE*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_UNTERMINATE* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_UNTERMINATE* "unterminate")
(intern "*CMD_NAME_WAKE*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_WAKE* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_WAKE* "wake")
(intern "*CMD_NAME_WAKE_SAFE*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_WAKE_SAFE* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_WAKE_SAFE* "wakeSafe")
(intern "*CMD_NAME_ARM_PAN_AND_TILT*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_ARM_PAN_AND_TILT* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_ARM_PAN_AND_TILT* "armPanAndTilt")
(intern "*CMD_NAME_DEPLOY_ARM*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_DEPLOY_ARM* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_DEPLOY_ARM* "deployArm")
(intern "*CMD_NAME_GRIPPER_CONTROL*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_GRIPPER_CONTROL* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_GRIPPER_CONTROL* "gripperControl")
(intern "*CMD_NAME_STOP_ARM*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_STOP_ARM* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_STOP_ARM* "stopArm")
(intern "*CMD_NAME_STOW_ARM*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_STOW_ARM* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_STOW_ARM* "stowArm")
(intern "*CMD_NAME_SET_DATA_TO_DISK*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_SET_DATA_TO_DISK* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_SET_DATA_TO_DISK* "setDataToDisk")
(intern "*CMD_NAME_START_RECORDING*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_START_RECORDING* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_START_RECORDING* "startRecording")
(intern "*CMD_NAME_STOP_RECORDING*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_STOP_RECORDING* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_STOP_RECORDING* "stopRecording")
(intern "*CMD_NAME_CUSTOM_GUEST_SCIENCE*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_CUSTOM_GUEST_SCIENCE* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_CUSTOM_GUEST_SCIENCE* "customGuestScience")
(intern "*CMD_NAME_START_GUEST_SCIENCE*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_START_GUEST_SCIENCE* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_START_GUEST_SCIENCE* "startGuestScience")
(intern "*CMD_NAME_STOP_GUEST_SCIENCE*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_STOP_GUEST_SCIENCE* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_STOP_GUEST_SCIENCE* "stopGuestScience")
(intern "*CMD_NAME_AUTO_RETURN*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_AUTO_RETURN* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_AUTO_RETURN* "autoReturn")
(intern "*CMD_NAME_DOCK*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_DOCK* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_DOCK* "dock")
(intern "*CMD_NAME_IDLE_PROPULSION*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_IDLE_PROPULSION* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_IDLE_PROPULSION* "idlePropulsion")
(intern "*CMD_NAME_PERCH*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_PERCH* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_PERCH* "perch")
(intern "*CMD_NAME_PREPARE*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_PREPARE* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_PREPARE* "prepare")
(intern "*CMD_NAME_SIMPLE_MOVE6DOF*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_SIMPLE_MOVE6DOF* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_SIMPLE_MOVE6DOF* "simpleMove6DOF")
(intern "*CMD_NAME_STOP_ALL_MOTION*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_STOP_ALL_MOTION* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_STOP_ALL_MOTION* "stopAllMotion")
(intern "*CMD_NAME_UNDOCK*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_UNDOCK* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_UNDOCK* "undock")
(intern "*CMD_NAME_UNPERCH*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_UNPERCH* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_UNPERCH* "unperch")
(intern "*CMD_NAME_PAUSE_PLAN*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_PAUSE_PLAN* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_PAUSE_PLAN* "pausePlan")
(intern "*CMD_NAME_RUN_PLAN*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_RUN_PLAN* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_RUN_PLAN* "runPlan")
(intern "*CMD_NAME_SET_PLAN*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_SET_PLAN* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_SET_PLAN* "setPlan")
(intern "*CMD_NAME_SKIP_PLAN_STEP*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_SKIP_PLAN_STEP* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_SKIP_PLAN_STEP* "skipPlanStep")
(intern "*CMD_NAME_WAIT*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_WAIT* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_WAIT* "wait")
(intern "*CMD_NAME_POWER_OFF_ITEM*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_POWER_OFF_ITEM* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_POWER_OFF_ITEM* "powerOffItem")
(intern "*CMD_NAME_POWER_ON_ITEM*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_POWER_ON_ITEM* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_POWER_ON_ITEM* "powerOnItem")
(intern "*CMD_NAME_SET_CAMERA*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_SET_CAMERA* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_SET_CAMERA* "setCamera")
(intern "*CMD_NAME_SET_CAMERA_RECORDING*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_SET_CAMERA_RECORDING* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_SET_CAMERA_RECORDING* "setCameraRecording")
(intern "*CMD_NAME_SET_CAMERA_STREAMING*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_SET_CAMERA_STREAMING* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_SET_CAMERA_STREAMING* "setCameraStreaming")
(intern "*CMD_NAME_SET_CHECK_OBSTACLES*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_SET_CHECK_OBSTACLES* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_SET_CHECK_OBSTACLES* "setCheckObstacles")
(intern "*CMD_NAME_SET_CHECK_ZONES*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_SET_CHECK_ZONES* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_SET_CHECK_ZONES* "setCheckZones")
(intern "*CMD_NAME_SET_ENABLE_AUTO_RETURN*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_SET_ENABLE_AUTO_RETURN* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_SET_ENABLE_AUTO_RETURN* "setEnableAutoReturn")
(intern "*CMD_NAME_SET_ENABLE_IMMEDIATE*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_SET_ENABLE_IMMEDIATE* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_SET_ENABLE_IMMEDIATE* "setEnableImmediate")
(intern "*CMD_NAME_SET_ENABLE_REPLAN*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_SET_ENABLE_REPLAN* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_SET_ENABLE_REPLAN* "setEnableReplan")
(intern "*CMD_NAME_SET_FLASHLIGHT_BRIGHTNESS*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_SET_FLASHLIGHT_BRIGHTNESS* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_SET_FLASHLIGHT_BRIGHTNESS* "setFlashlightBrightness")
(intern "*CMD_NAME_SET_HOLONOMIC_MODE*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_SET_HOLONOMIC_MODE* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_SET_HOLONOMIC_MODE* "setHolonomicMode")
(intern "*CMD_NAME_SET_INERTIA*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_SET_INERTIA* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_SET_INERTIA* "setInertia")
(intern "*CMD_NAME_SET_OPERATING_LIMITS*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_SET_OPERATING_LIMITS* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_SET_OPERATING_LIMITS* "setOperatingLimits")
(intern "*CMD_NAME_SET_PLANNER*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_SET_PLANNER* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_SET_PLANNER* "setPlanner")
(intern "*CMD_NAME_SET_TELEMETRY_RATE*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_SET_TELEMETRY_RATE* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_SET_TELEMETRY_RATE* "setTelemetryRate")
(intern "*CMD_NAME_SET_ZONES*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_NAME_SET_ZONES* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_NAME_SET_ZONES* "setZones")
(intern "*CMD_SUBSYS_ACCESS_CONTROL*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_SUBSYS_ACCESS_CONTROL* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_SUBSYS_ACCESS_CONTROL* "AccessControl")
(intern "*CMD_SUBSYS_ADMIN*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_SUBSYS_ADMIN* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_SUBSYS_ADMIN* "Admin")
(intern "*CMD_SUBSYS_ARM*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_SUBSYS_ARM* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_SUBSYS_ARM* "Arm")
(intern "*CMD_SUBSYS_DATA*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_SUBSYS_DATA* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_SUBSYS_DATA* "Data")
(intern "*CMD_SUBSYS_GUEST_SCIENCE*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_SUBSYS_GUEST_SCIENCE* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_SUBSYS_GUEST_SCIENCE* "GuestScience")
(intern "*CMD_SUBSYS_MOBILITY*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_SUBSYS_MOBILITY* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_SUBSYS_MOBILITY* "Mobility")
(intern "*CMD_SUBSYS_PLAN*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_SUBSYS_PLAN* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_SUBSYS_PLAN* "Plan")
(intern "*CMD_SUBSYS_POWER*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_SUBSYS_POWER* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_SUBSYS_POWER* "Power")
(intern "*CMD_SUBSYS_SETTINGS*" (find-package "FF_MSGS::COMMANDCONSTANTS"))
(shadow '*CMD_SUBSYS_SETTINGS* (find-package "FF_MSGS::COMMANDCONSTANTS"))
(defconstant ff_msgs::CommandConstants::*CMD_SUBSYS_SETTINGS* "Settings")
(defclass ff_msgs::CommandConstants
  :super ros::object
  :slots ())

(defmethod ff_msgs::CommandConstants
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get ff_msgs::CommandConstants :md5sum-) "f1f0fb85668017d10454dc888758cac6")
(setf (get ff_msgs::CommandConstants :datatype-) "ff_msgs/CommandConstants")
(setf (get ff_msgs::CommandConstants :definition-)
      "# Copyright (c) 2015 United States Government as represented by the
# Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.

string PARAM_NAME_LOCALIZATION_MODE_NONE = None
string PARAM_NAME_LOCALIZATION_MODE_MAPPED_LANDMARKS = MappedLandmarks
string PARAM_NAME_LOCALIZATION_MODE_ARTAGS = ARTags
string PARAM_NAME_LOCALIZATION_MODE_HANDRAIL = Handrail
string PARAM_NAME_LOCALIZATION_MODE_PERCH = Perch
string PARAM_NAME_LOCALIZATION_MODE_TRUTH = Truth
string PARAM_NAME_ACTION_TYPE_PAN = Pan
string PARAM_NAME_ACTION_TYPE_TILT = Tilt
string PARAM_NAME_ACTION_TYPE_BOTH = Both
string PARAM_NAME_POWERED_COMPONENT_LASER_POINTER = Laser Pointer
string PARAM_NAME_POWERED_COMPONENT_PAYLOAD_TOP_AFT = Payload Top Aft
string PARAM_NAME_POWERED_COMPONENT_PAYLOAD_BOTTOM_AFT = Payload Bottom Aft
string PARAM_NAME_POWERED_COMPONENT_PAYLOAD_BOTTOM_FRONT = Payload Bottom Front
string PARAM_NAME_POWERED_COMPONENT_PMCS_AND_SIGNAL_LIGHTS = PMC
string PARAM_NAME_CAMERA_MODE_BOTH = Both
string PARAM_NAME_CAMERA_MODE_RECORDING = Recording
string PARAM_NAME_CAMERA_MODE_STREAMING = Streaming
string PARAM_NAME_CAMERA_NAME_SCI = Science
string PARAM_NAME_CAMERA_NAME_NAV = Navigation
string PARAM_NAME_CAMERA_NAME_HAZ = Hazard
string PARAM_NAME_CAMERA_NAME_DOCK = Dock
string PARAM_NAME_CAMERA_NAME_PERCH = Perch
string PARAM_NAME_CAMERA_RESOLUTION_224X171 = 224x171
string PARAM_NAME_CAMERA_RESOLUTION_320X240 = 320x240
string PARAM_NAME_CAMERA_RESOLUTION_480X270 = 480x270
string PARAM_NAME_CAMERA_RESOLUTION_640X480 = 640x480
string PARAM_NAME_CAMERA_RESOLUTION_960X540 = 960x540
string PARAM_NAME_CAMERA_RESOLUTION_1024X768 = 1024x768
string PARAM_NAME_CAMERA_RESOLUTION_1280X720 = 1280x720
string PARAM_NAME_CAMERA_RESOLUTION_1280X960 = 1280x960
string PARAM_NAME_CAMERA_RESOLUTION_1920X1080 = 1920x1080
string PARAM_NAME_FLASHLIGHT_LOCATION_BACK = Back
string PARAM_NAME_FLASHLIGHT_LOCATION_FRONT = Front
string PARAM_NAME_FLIGHT_MODE_OFF = off
string PARAM_NAME_FLIGHT_MODE_QUIET = quiet
string PARAM_NAME_FLIGHT_MODE_NOMINAL = nominal
string PARAM_NAME_FLIGHT_MODE_DIFFICULT = difficult
string PARAM_NAME_FLIGHT_MODE_PRECISION = precision
string PARAM_NAME_PLANNER_TYPE_TRAPEZOIDAL = trapezoidal
string PARAM_NAME_PLANNER_TYPE_QUADRATIC_PROGRAM = qp
string PARAM_NAME_TELEMETRY_TYPE_COMM_STATUS = CommStatus
string PARAM_NAME_TELEMETRY_TYPE_CPU_STATE = CpuState
string PARAM_NAME_TELEMETRY_TYPE_DISK_STATE = DiskState
string PARAM_NAME_TELEMETRY_TYPE_EKF_STATE = EkfState
string PARAM_NAME_TELEMETRY_TYPE_GNC_STATE = GncState
string PARAM_NAME_TELEMETRY_TYPE_PMC_CMD_STATE = PmcCmdState
string PARAM_NAME_TELEMETRY_TYPE_POSITION = Position
string PARAM_NAME_TELEMETRY_TYPE_SPARSE_MAPPING_POSE = SparseMappingPose

string CMD_NAME_GRAB_CONTROL = grabControl
string CMD_NAME_REQUEST_CONTROL = requestControl
string CMD_NAME_FAULT = fault
string CMD_NAME_INITIALIZE_BIAS = initializeBias
string CMD_NAME_LOAD_NODELET = loadNodelet
string CMD_NAME_NO_OP = noOp
string CMD_NAME_REACQUIRE_POSITION = reacquirePosition
string CMD_NAME_RESET_EKF = resetEkf
string CMD_NAME_SWITCH_LOCALIZATION = switchLocalization
string CMD_NAME_UNLOAD_NODELET = unloadNodelet
string CMD_NAME_UNTERMINATE = unterminate
string CMD_NAME_WAKE = wake
string CMD_NAME_WAKE_SAFE = wakeSafe
string CMD_NAME_ARM_PAN_AND_TILT = armPanAndTilt
string CMD_NAME_DEPLOY_ARM = deployArm
string CMD_NAME_GRIPPER_CONTROL = gripperControl
string CMD_NAME_STOP_ARM = stopArm
string CMD_NAME_STOW_ARM = stowArm
string CMD_NAME_SET_DATA_TO_DISK = setDataToDisk
string CMD_NAME_START_RECORDING = startRecording
string CMD_NAME_STOP_RECORDING = stopRecording
string CMD_NAME_CUSTOM_GUEST_SCIENCE = customGuestScience
string CMD_NAME_START_GUEST_SCIENCE = startGuestScience
string CMD_NAME_STOP_GUEST_SCIENCE = stopGuestScience
string CMD_NAME_AUTO_RETURN = autoReturn
string CMD_NAME_DOCK = dock
string CMD_NAME_IDLE_PROPULSION = idlePropulsion
string CMD_NAME_PERCH = perch
string CMD_NAME_PREPARE = prepare
string CMD_NAME_SIMPLE_MOVE6DOF = simpleMove6DOF
string CMD_NAME_STOP_ALL_MOTION = stopAllMotion
string CMD_NAME_UNDOCK = undock
string CMD_NAME_UNPERCH = unperch
string CMD_NAME_PAUSE_PLAN = pausePlan
string CMD_NAME_RUN_PLAN = runPlan
string CMD_NAME_SET_PLAN = setPlan
string CMD_NAME_SKIP_PLAN_STEP = skipPlanStep
string CMD_NAME_WAIT = wait
string CMD_NAME_POWER_OFF_ITEM = powerOffItem
string CMD_NAME_POWER_ON_ITEM = powerOnItem
string CMD_NAME_SET_CAMERA = setCamera
string CMD_NAME_SET_CAMERA_RECORDING = setCameraRecording
string CMD_NAME_SET_CAMERA_STREAMING = setCameraStreaming
string CMD_NAME_SET_CHECK_OBSTACLES = setCheckObstacles
string CMD_NAME_SET_CHECK_ZONES = setCheckZones
string CMD_NAME_SET_ENABLE_AUTO_RETURN = setEnableAutoReturn
string CMD_NAME_SET_ENABLE_IMMEDIATE = setEnableImmediate
string CMD_NAME_SET_ENABLE_REPLAN = setEnableReplan
string CMD_NAME_SET_FLASHLIGHT_BRIGHTNESS = setFlashlightBrightness
string CMD_NAME_SET_HOLONOMIC_MODE = setHolonomicMode
string CMD_NAME_SET_INERTIA = setInertia
string CMD_NAME_SET_OPERATING_LIMITS = setOperatingLimits
string CMD_NAME_SET_PLANNER = setPlanner
string CMD_NAME_SET_TELEMETRY_RATE = setTelemetryRate
string CMD_NAME_SET_ZONES = setZones

string CMD_SUBSYS_ACCESS_CONTROL = AccessControl
string CMD_SUBSYS_ADMIN = Admin
string CMD_SUBSYS_ARM = Arm
string CMD_SUBSYS_DATA = Data
string CMD_SUBSYS_GUEST_SCIENCE = GuestScience
string CMD_SUBSYS_MOBILITY = Mobility
string CMD_SUBSYS_PLAN = Plan
string CMD_SUBSYS_POWER = Power
string CMD_SUBSYS_SETTINGS = Settings

")



(provide :ff_msgs/CommandConstants "f1f0fb85668017d10454dc888758cac6")


