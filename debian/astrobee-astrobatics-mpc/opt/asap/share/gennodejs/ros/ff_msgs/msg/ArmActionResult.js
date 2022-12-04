// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ArmResult = require('./ArmResult.js');
let actionlib_msgs = _finder('actionlib_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ArmActionResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.status = null;
      this.result = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = new actionlib_msgs.msg.GoalStatus();
      }
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = new ArmResult();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ArmActionResult
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = actionlib_msgs.msg.GoalStatus.serialize(obj.status, buffer, bufferOffset);
    // Serialize message field [result]
    bufferOffset = ArmResult.serialize(obj.result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArmActionResult
    let len;
    let data = new ArmActionResult(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = actionlib_msgs.msg.GoalStatus.deserialize(buffer, bufferOffset);
    // Deserialize message field [result]
    data.result = ArmResult.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += actionlib_msgs.msg.GoalStatus.getMessageSize(object.status);
    length += ArmResult.getMessageSize(object.result);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/ArmActionResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b079a56e922752b036e0c53789575ff3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    
    Header header
    actionlib_msgs/GoalStatus status
    ArmResult result
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    ================================================================================
    MSG: actionlib_msgs/GoalStatus
    GoalID goal_id
    uint8 status
    uint8 PENDING         = 0   # The goal has yet to be processed by the action server
    uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
    uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                                #   and has since completed its execution (Terminal State)
    uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
    uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                                #    to some failure (Terminal State)
    uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                                #    because the goal was unattainable or invalid (Terminal State)
    uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                                #    and has not yet completed execution
    uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                                #    but the action server has not yet confirmed that the goal is canceled
    uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                                #    and was successfully cancelled (Terminal State)
    uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                                #    sent over the wire by an action server
    
    #Allow for the user to associate a string with GoalStatus for debugging
    string text
    
    
    ================================================================================
    MSG: actionlib_msgs/GoalID
    # The stamp should store the time at which this goal was requested.
    # It is used by an action server when it tries to preempt all
    # goals that were requested before a certain time
    time stamp
    
    # The id provides a way to associate feedback and
    # result message with specific goal requests. The id
    # specified must be unique.
    string id
    
    
    ================================================================================
    MSG: ff_msgs/ArmResult
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
    const resolved = new ArmActionResult(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.status !== undefined) {
      resolved.status = actionlib_msgs.msg.GoalStatus.Resolve(msg.status)
    }
    else {
      resolved.status = new actionlib_msgs.msg.GoalStatus()
    }

    if (msg.result !== undefined) {
      resolved.result = ArmResult.Resolve(msg.result)
    }
    else {
      resolved.result = new ArmResult()
    }

    return resolved;
    }
};

module.exports = ArmActionResult;