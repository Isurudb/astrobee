// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class PerchState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.state = null;
      this.fsm_event = null;
      this.fsm_state = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = 0;
      }
      if (initObj.hasOwnProperty('fsm_event')) {
        this.fsm_event = initObj.fsm_event
      }
      else {
        this.fsm_event = '';
      }
      if (initObj.hasOwnProperty('fsm_state')) {
        this.fsm_state = initObj.fsm_state
      }
      else {
        this.fsm_state = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PerchState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.int8(obj.state, buffer, bufferOffset);
    // Serialize message field [fsm_event]
    bufferOffset = _serializer.string(obj.fsm_event, buffer, bufferOffset);
    // Serialize message field [fsm_state]
    bufferOffset = _serializer.string(obj.fsm_state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PerchState
    let len;
    let data = new PerchState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [fsm_event]
    data.fsm_event = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [fsm_state]
    data.fsm_state = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.fsm_event.length;
    length += object.fsm_state.length;
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/PerchState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5a3bc3d43070c3cb7655a2601ea68801';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Copyright (c) 2017, United States Government, as represented by the
    # Administrator of the National Aeronautics and Space Administration.
    #
    # All rights reserved.
    #
    # The Astrobee platform is licensed under the Apache License, Version 2.0
    # (the "License"); you may not use this file except in compliance with the
    # License. You may obtain a copy of the License at
    #
    #     http://www.apache.org/licenses/LICENSE-2.0
    #
    # Unless required by applicable law or agreed to in writing, software
    # distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
    # WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
    # License for the specific language governing permissions and limitations
    # under the License.
    #
    # The state of the perching system
    
    # Header with timestamp
    std_msgs/Header header
    
    # Feedback
    int8 state
    
    int8 RECOVERY_MOVING_TO_RECOVERY_POSE   = 18
    int8 RECOVERY_SWITCHING_TO_ML_LOC       = 17
    int8 RECOVERY_STOWING_ARM               = 16
    int8 RECOVERY_MOVING_TO_APPROACH_POSE   = 15
    int8 RECOVERY_OPENING_GRIPPER           = 14
    int8 INITIALIZING                       = 13
    int8 UNKNOWN                            = 12
    # Used to check the perching/unperching ranges
    int8 PERCHING_MAX_STATE                 = 11
    int8 PERCHING_SWITCHING_TO_HR_LOC       = 11
    int8 PERCHING_MOVING_TO_APPROACH_POSE   = 10
    int8 PERCHING_ENSURING_APPROACH_POSE    = 9
    int8 PERCHING_DEPLOYING_ARM             = 8
    int8 PERCHING_OPENING_GRIPPER           = 7
    int8 PERCHING_MOVING_TO_COMPLETE_POSE   = 6
    int8 PERCHING_CLOSING_GRIPPER           = 5
    int8 PERCHING_CHECKING_ATTACHED         = 4
    int8 PERCHING_WAITING_FOR_SPIN_DOWN     = 3
    int8 PERCHING_SWITCHING_TO_PL_LOC       = 2
    int8 PERCHING_STOPPING                  = 1
    int8 PERCHED                            = 0
    int8 UNPERCHING_SWITCHING_TO_HR_LOC     = -1
    int8 UNPERCHING_WAITING_FOR_SPIN_UP     = -2
    int8 UNPERCHING_OPENING_GRIPPER         = -3
    int8 UNPERCHING_MOVING_TO_APPROACH_POSE = -4
    int8 UNPERCHING_STOWING_ARM             = -5
    int8 UNPERCHING_SWITCHING_TO_ML_LOC     = -6
    int8 UNPERCHED                          = -7
    # Used to check the perching/unperching ranges
    int8 UNPERCHING_MAX_STATE               = -7
    
    # A human readable version of the (event) -> [state] transition
    string fsm_event
    string fsm_state
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PerchState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = 0
    }

    if (msg.fsm_event !== undefined) {
      resolved.fsm_event = msg.fsm_event;
    }
    else {
      resolved.fsm_event = ''
    }

    if (msg.fsm_state !== undefined) {
      resolved.fsm_state = msg.fsm_state;
    }
    else {
      resolved.fsm_state = ''
    }

    return resolved;
    }
};

// Constants for message
PerchState.Constants = {
  RECOVERY_MOVING_TO_RECOVERY_POSE: 18,
  RECOVERY_SWITCHING_TO_ML_LOC: 17,
  RECOVERY_STOWING_ARM: 16,
  RECOVERY_MOVING_TO_APPROACH_POSE: 15,
  RECOVERY_OPENING_GRIPPER: 14,
  INITIALIZING: 13,
  UNKNOWN: 12,
  PERCHING_MAX_STATE: 11,
  PERCHING_SWITCHING_TO_HR_LOC: 11,
  PERCHING_MOVING_TO_APPROACH_POSE: 10,
  PERCHING_ENSURING_APPROACH_POSE: 9,
  PERCHING_DEPLOYING_ARM: 8,
  PERCHING_OPENING_GRIPPER: 7,
  PERCHING_MOVING_TO_COMPLETE_POSE: 6,
  PERCHING_CLOSING_GRIPPER: 5,
  PERCHING_CHECKING_ATTACHED: 4,
  PERCHING_WAITING_FOR_SPIN_DOWN: 3,
  PERCHING_SWITCHING_TO_PL_LOC: 2,
  PERCHING_STOPPING: 1,
  PERCHED: 0,
  UNPERCHING_SWITCHING_TO_HR_LOC: -1,
  UNPERCHING_WAITING_FOR_SPIN_UP: -2,
  UNPERCHING_OPENING_GRIPPER: -3,
  UNPERCHING_MOVING_TO_APPROACH_POSE: -4,
  UNPERCHING_STOWING_ARM: -5,
  UNPERCHING_SWITCHING_TO_ML_LOC: -6,
  UNPERCHED: -7,
  UNPERCHING_MAX_STATE: -7,
}

module.exports = PerchState;
