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

class DockState {
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
    // Serializes a message object of type DockState
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
    //deserializes a message object of type DockState
    let len;
    let data = new DockState(null);
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
    return 'ff_msgs/DockState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '09d842ed9109dbc2bf96976f8fb2eaa5';
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
    # Response for Dock/Undock goals
    
    # Header with timestamp
    std_msgs/Header header
    
    # Feedback
    int8 state
    int8 RECOVERY_SWITCHING_TO_ML_LOC       = 15
    int8 RECOVERY_MOVING_TO_APPROACH_POSE   = 14
    int8 RECOVERY_WAITING_FOR_SPIN_DOWN     = 13
    int8 RECOVERY_SWITCHING_TO_NO_LOC       = 12
    int8 INITIALIZING                       = 11
    int8 UNKNOWN                            = 10
    int8 DOCKING_MAX_STATE                  = 7
    int8 DOCKING_SWITCHING_TO_ML_LOC        = 7
    int8 DOCKING_MOVING_TO_APPROACH_POSE    = 6
    int8 DOCKING_SWITCHING_TO_AR_LOC        = 5
    int8 DOCKING_MOVING_TO_COMPLETE_POSE    = 4
    int8 DOCKING_CHECKING_ATTACHED          = 3
    int8 DOCKING_WAITING_FOR_SPIN_DOWN      = 2
    int8 DOCKING_SWITCHING_TO_NO_LOC        = 1
    int8 DOCKED                             = 0
    int8 UNDOCKING_SWITCHING_TO_ML_LOC      = -1
    int8 UNDOCKING_WAITING_FOR_SPIN_UP      = -2
    int8 UNDOCKING_MOVING_TO_APPROACH_POSE  = -3
    int8 UNDOCKED                           = -4
    int8 UNDOCKING_MAX_STATE                = -4
    
    # A human readble version of the (event) -> [state] transition
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
    const resolved = new DockState(null);
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
DockState.Constants = {
  RECOVERY_SWITCHING_TO_ML_LOC: 15,
  RECOVERY_MOVING_TO_APPROACH_POSE: 14,
  RECOVERY_WAITING_FOR_SPIN_DOWN: 13,
  RECOVERY_SWITCHING_TO_NO_LOC: 12,
  INITIALIZING: 11,
  UNKNOWN: 10,
  DOCKING_MAX_STATE: 7,
  DOCKING_SWITCHING_TO_ML_LOC: 7,
  DOCKING_MOVING_TO_APPROACH_POSE: 6,
  DOCKING_SWITCHING_TO_AR_LOC: 5,
  DOCKING_MOVING_TO_COMPLETE_POSE: 4,
  DOCKING_CHECKING_ATTACHED: 3,
  DOCKING_WAITING_FOR_SPIN_DOWN: 2,
  DOCKING_SWITCHING_TO_NO_LOC: 1,
  DOCKED: 0,
  UNDOCKING_SWITCHING_TO_ML_LOC: -1,
  UNDOCKING_WAITING_FOR_SPIN_UP: -2,
  UNDOCKING_MOVING_TO_APPROACH_POSE: -3,
  UNDOCKED: -4,
  UNDOCKING_MAX_STATE: -4,
}

module.exports = DockState;
