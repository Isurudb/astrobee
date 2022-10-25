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

class DockResult {
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
    // Serializes a message object of type DockResult
    // Serialize message field [response]
    bufferOffset = _serializer.int32(obj.response, buffer, bufferOffset);
    // Serialize message field [fsm_result]
    bufferOffset = _serializer.string(obj.fsm_result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DockResult
    let len;
    let data = new DockResult(null);
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
    return 'ff_msgs/DockResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0cc69ac3a301c7996578d2ee3e9b92a6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    
    # Result
    int32 response
    int32 CANCELLED                          = 5
    int32 ALREADY_DOCKED                     = 4
    int32 ALREADY_UNDOCKED                   = 3
    int32 UNDOCKED                           = 2
    int32 DOCKED                             = 1
    int32 PREEMPTED                          = 0
    int32 INVALID_COMMAND                    = -1
    int32 INVALID_BERTH                      = -2
    int32 NOT_IN_UNDOCKED_STATE              = -3
    int32 NOT_IN_DOCKED_STATE                = -4
    int32 SWITCH_TO_ML_FAILED                = -5
    int32 SWITCH_TO_AR_FAILED                = -6
    int32 SWITCH_TO_NO_FAILED                = -7
    int32 PREP_DISABLE_FAILED                = -8
    int32 PREP_ENABLE_FAILED                 = -9
    int32 MOTION_APPROACH_FAILED             = -10
    int32 MOTION_COMPLETE_FAILED             = -11
    int32 MOTION_ATTACHED_FAILED             = -12
    int32 EPS_UNDOCK_FAILED                  = -13
    int32 EPS_DOCK_FAILED                    = -14
    int32 TOO_FAR_AWAY_FROM_APPROACH         = -15
    
    # Human readable FSM result for debugging
    string fsm_result
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DockResult(null);
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
DockResult.Constants = {
  CANCELLED: 5,
  ALREADY_DOCKED: 4,
  ALREADY_UNDOCKED: 3,
  UNDOCKED: 2,
  DOCKED: 1,
  PREEMPTED: 0,
  INVALID_COMMAND: -1,
  INVALID_BERTH: -2,
  NOT_IN_UNDOCKED_STATE: -3,
  NOT_IN_DOCKED_STATE: -4,
  SWITCH_TO_ML_FAILED: -5,
  SWITCH_TO_AR_FAILED: -6,
  SWITCH_TO_NO_FAILED: -7,
  PREP_DISABLE_FAILED: -8,
  PREP_ENABLE_FAILED: -9,
  MOTION_APPROACH_FAILED: -10,
  MOTION_COMPLETE_FAILED: -11,
  MOTION_ATTACHED_FAILED: -12,
  EPS_UNDOCK_FAILED: -13,
  EPS_DOCK_FAILED: -14,
  TOO_FAR_AWAY_FROM_APPROACH: -15,
}

module.exports = DockResult;
