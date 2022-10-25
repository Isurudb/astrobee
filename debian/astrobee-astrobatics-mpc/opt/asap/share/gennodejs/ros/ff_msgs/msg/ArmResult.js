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

class ArmResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.response = null;
      this.fsm_result = null;
    }
    else {
      if (initObj.hasOwnProperty('response')) {
        this.response = initObj.response
      }
      else {
        this.response = 0;
      }
      if (initObj.hasOwnProperty('fsm_result')) {
        this.fsm_result = initObj.fsm_result
      }
      else {
        this.fsm_result = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ArmResult
    // Serialize message field [response]
    bufferOffset = _serializer.int32(obj.response, buffer, bufferOffset);
    // Serialize message field [fsm_result]
    bufferOffset = _serializer.string(obj.fsm_result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArmResult
    let len;
    let data = new ArmResult(null);
    // Deserialize message field [response]
    data.response = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [fsm_result]
    data.fsm_result = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.fsm_result.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/ArmResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5c229b93f1064b9f1f7e8f3320eff359';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    
    # Machine-readable reseult code
    int32 response
    int32 SUCCESS             =  1                # Successfully completed
    int32 PREEMPTED           =  0                # Action was preempted
    int32 INVALID_COMMAND     = -1                # Invalid command
    int32 BAD_TILT_VALUE      = -2                # Invalid value for tilt
    int32 BAD_PAN_VALUE       = -3                # Invalid value for pan
    int32 BAD_GRIPPER_VALUE   = -4                # Invalid value for gripper
    int32 NOT_ALLOWED         = -5                # Not allowed
    int32 TILT_FAILED         = -6                # Tilt command failed
    int32 PAN_FAILED          = -7                # Pan command failed
    int32 GRIPPER_FAILED      = -8                # Gripper command failed
    int32 COMMUNICATION_ERROR = -9                # Cannot communicate with arm
    int32 COLLISION_AVOIDED   = -10               # No panning when tilt < 90
    int32 ENABLE_FAILED       = -11               # Cannot enable the servos
    int32 DISABLE_FAILED      = -12               # Cannot disable the servos
    int32 CALIBRATE_FAILED    = -13               # Cannot calibrate the gripper
    int32 NO_GOAL             = -14               # Unknown call to calibration
    
    # Human readable FSM result for debugging
    string fsm_result
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ArmResult(null);
    if (msg.response !== undefined) {
      resolved.response = msg.response;
    }
    else {
      resolved.response = 0
    }

    if (msg.fsm_result !== undefined) {
      resolved.fsm_result = msg.fsm_result;
    }
    else {
      resolved.fsm_result = ''
    }

    return resolved;
    }
};

// Constants for message
ArmResult.Constants = {
  SUCCESS: 1,
  PREEMPTED: 0,
  INVALID_COMMAND: -1,
  BAD_TILT_VALUE: -2,
  BAD_PAN_VALUE: -3,
  BAD_GRIPPER_VALUE: -4,
  NOT_ALLOWED: -5,
  TILT_FAILED: -6,
  PAN_FAILED: -7,
  GRIPPER_FAILED: -8,
  COMMUNICATION_ERROR: -9,
  COLLISION_AVOIDED: -10,
  ENABLE_FAILED: -11,
  DISABLE_FAILED: -12,
  CALIBRATE_FAILED: -13,
  NO_GOAL: -14,
}

module.exports = ArmResult;
