// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class CommandConstants {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CommandConstants
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CommandConstants
    let len;
    let data = new CommandConstants(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/CommandConstants';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f1f0fb85668017d10454dc888758cac6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Copyright (c) 2015 United States Government as represented by the
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CommandConstants(null);
    return resolved;
    }
};

// Constants for message
CommandConstants.Constants = {
  PARAM_NAME_LOCALIZATION_MODE_NONE: 'None',
  PARAM_NAME_LOCALIZATION_MODE_MAPPED_LANDMARKS: 'MappedLandmarks',
  PARAM_NAME_LOCALIZATION_MODE_ARTAGS: 'ARTags',
  PARAM_NAME_LOCALIZATION_MODE_HANDRAIL: 'Handrail',
  PARAM_NAME_LOCALIZATION_MODE_PERCH: 'Perch',
  PARAM_NAME_LOCALIZATION_MODE_TRUTH: 'Truth',
  PARAM_NAME_ACTION_TYPE_PAN: 'Pan',
  PARAM_NAME_ACTION_TYPE_TILT: 'Tilt',
  PARAM_NAME_ACTION_TYPE_BOTH: 'Both',
  PARAM_NAME_POWERED_COMPONENT_LASER_POINTER: 'Laser Pointer',
  PARAM_NAME_POWERED_COMPONENT_PAYLOAD_TOP_AFT: 'Payload Top Aft',
  PARAM_NAME_POWERED_COMPONENT_PAYLOAD_BOTTOM_AFT: 'Payload Bottom Aft',
  PARAM_NAME_POWERED_COMPONENT_PAYLOAD_BOTTOM_FRONT: 'Payload Bottom Front',
  PARAM_NAME_POWERED_COMPONENT_PMCS_AND_SIGNAL_LIGHTS: 'PMC',
  PARAM_NAME_CAMERA_MODE_BOTH: 'Both',
  PARAM_NAME_CAMERA_MODE_RECORDING: 'Recording',
  PARAM_NAME_CAMERA_MODE_STREAMING: 'Streaming',
  PARAM_NAME_CAMERA_NAME_SCI: 'Science',
  PARAM_NAME_CAMERA_NAME_NAV: 'Navigation',
  PARAM_NAME_CAMERA_NAME_HAZ: 'Hazard',
  PARAM_NAME_CAMERA_NAME_DOCK: 'Dock',
  PARAM_NAME_CAMERA_NAME_PERCH: 'Perch',
  PARAM_NAME_CAMERA_RESOLUTION_224X171: '224x171',
  PARAM_NAME_CAMERA_RESOLUTION_320X240: '320x240',
  PARAM_NAME_CAMERA_RESOLUTION_480X270: '480x270',
  PARAM_NAME_CAMERA_RESOLUTION_640X480: '640x480',
  PARAM_NAME_CAMERA_RESOLUTION_960X540: '960x540',
  PARAM_NAME_CAMERA_RESOLUTION_1024X768: '1024x768',
  PARAM_NAME_CAMERA_RESOLUTION_1280X720: '1280x720',
  PARAM_NAME_CAMERA_RESOLUTION_1280X960: '1280x960',
  PARAM_NAME_CAMERA_RESOLUTION_1920X1080: '1920x1080',
  PARAM_NAME_FLASHLIGHT_LOCATION_BACK: 'Back',
  PARAM_NAME_FLASHLIGHT_LOCATION_FRONT: 'Front',
  PARAM_NAME_FLIGHT_MODE_OFF: 'off',
  PARAM_NAME_FLIGHT_MODE_QUIET: 'quiet',
  PARAM_NAME_FLIGHT_MODE_NOMINAL: 'nominal',
  PARAM_NAME_FLIGHT_MODE_DIFFICULT: 'difficult',
  PARAM_NAME_FLIGHT_MODE_PRECISION: 'precision',
  PARAM_NAME_PLANNER_TYPE_TRAPEZOIDAL: 'trapezoidal',
  PARAM_NAME_PLANNER_TYPE_QUADRATIC_PROGRAM: 'qp',
  PARAM_NAME_TELEMETRY_TYPE_COMM_STATUS: 'CommStatus',
  PARAM_NAME_TELEMETRY_TYPE_CPU_STATE: 'CpuState',
  PARAM_NAME_TELEMETRY_TYPE_DISK_STATE: 'DiskState',
  PARAM_NAME_TELEMETRY_TYPE_EKF_STATE: 'EkfState',
  PARAM_NAME_TELEMETRY_TYPE_GNC_STATE: 'GncState',
  PARAM_NAME_TELEMETRY_TYPE_PMC_CMD_STATE: 'PmcCmdState',
  PARAM_NAME_TELEMETRY_TYPE_POSITION: 'Position',
  PARAM_NAME_TELEMETRY_TYPE_SPARSE_MAPPING_POSE: 'SparseMappingPose',
  CMD_NAME_GRAB_CONTROL: 'grabControl',
  CMD_NAME_REQUEST_CONTROL: 'requestControl',
  CMD_NAME_FAULT: 'fault',
  CMD_NAME_INITIALIZE_BIAS: 'initializeBias',
  CMD_NAME_LOAD_NODELET: 'loadNodelet',
  CMD_NAME_NO_OP: 'noOp',
  CMD_NAME_REACQUIRE_POSITION: 'reacquirePosition',
  CMD_NAME_RESET_EKF: 'resetEkf',
  CMD_NAME_SWITCH_LOCALIZATION: 'switchLocalization',
  CMD_NAME_UNLOAD_NODELET: 'unloadNodelet',
  CMD_NAME_UNTERMINATE: 'unterminate',
  CMD_NAME_WAKE: 'wake',
  CMD_NAME_WAKE_SAFE: 'wakeSafe',
  CMD_NAME_ARM_PAN_AND_TILT: 'armPanAndTilt',
  CMD_NAME_DEPLOY_ARM: 'deployArm',
  CMD_NAME_GRIPPER_CONTROL: 'gripperControl',
  CMD_NAME_STOP_ARM: 'stopArm',
  CMD_NAME_STOW_ARM: 'stowArm',
  CMD_NAME_SET_DATA_TO_DISK: 'setDataToDisk',
  CMD_NAME_START_RECORDING: 'startRecording',
  CMD_NAME_STOP_RECORDING: 'stopRecording',
  CMD_NAME_CUSTOM_GUEST_SCIENCE: 'customGuestScience',
  CMD_NAME_START_GUEST_SCIENCE: 'startGuestScience',
  CMD_NAME_STOP_GUEST_SCIENCE: 'stopGuestScience',
  CMD_NAME_AUTO_RETURN: 'autoReturn',
  CMD_NAME_DOCK: 'dock',
  CMD_NAME_IDLE_PROPULSION: 'idlePropulsion',
  CMD_NAME_PERCH: 'perch',
  CMD_NAME_PREPARE: 'prepare',
  CMD_NAME_SIMPLE_MOVE6DOF: 'simpleMove6DOF',
  CMD_NAME_STOP_ALL_MOTION: 'stopAllMotion',
  CMD_NAME_UNDOCK: 'undock',
  CMD_NAME_UNPERCH: 'unperch',
  CMD_NAME_PAUSE_PLAN: 'pausePlan',
  CMD_NAME_RUN_PLAN: 'runPlan',
  CMD_NAME_SET_PLAN: 'setPlan',
  CMD_NAME_SKIP_PLAN_STEP: 'skipPlanStep',
  CMD_NAME_WAIT: 'wait',
  CMD_NAME_POWER_OFF_ITEM: 'powerOffItem',
  CMD_NAME_POWER_ON_ITEM: 'powerOnItem',
  CMD_NAME_SET_CAMERA: 'setCamera',
  CMD_NAME_SET_CAMERA_RECORDING: 'setCameraRecording',
  CMD_NAME_SET_CAMERA_STREAMING: 'setCameraStreaming',
  CMD_NAME_SET_CHECK_OBSTACLES: 'setCheckObstacles',
  CMD_NAME_SET_CHECK_ZONES: 'setCheckZones',
  CMD_NAME_SET_ENABLE_AUTO_RETURN: 'setEnableAutoReturn',
  CMD_NAME_SET_ENABLE_IMMEDIATE: 'setEnableImmediate',
  CMD_NAME_SET_ENABLE_REPLAN: 'setEnableReplan',
  CMD_NAME_SET_FLASHLIGHT_BRIGHTNESS: 'setFlashlightBrightness',
  CMD_NAME_SET_HOLONOMIC_MODE: 'setHolonomicMode',
  CMD_NAME_SET_INERTIA: 'setInertia',
  CMD_NAME_SET_OPERATING_LIMITS: 'setOperatingLimits',
  CMD_NAME_SET_PLANNER: 'setPlanner',
  CMD_NAME_SET_TELEMETRY_RATE: 'setTelemetryRate',
  CMD_NAME_SET_ZONES: 'setZones',
  CMD_SUBSYS_ACCESS_CONTROL: 'AccessControl',
  CMD_SUBSYS_ADMIN: 'Admin',
  CMD_SUBSYS_ARM: 'Arm',
  CMD_SUBSYS_DATA: 'Data',
  CMD_SUBSYS_GUEST_SCIENCE: 'GuestScience',
  CMD_SUBSYS_MOBILITY: 'Mobility',
  CMD_SUBSYS_PLAN: 'Plan',
  CMD_SUBSYS_POWER: 'Power',
  CMD_SUBSYS_SETTINGS: 'Settings',
}

module.exports = CommandConstants;
