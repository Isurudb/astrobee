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

class ArmState {
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
    // Serializes a message object of type ArmState
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
    //deserializes a message object of type ArmState
    let len;
    let data = new ArmState(null);
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
    return 'ff_msgs/ArmState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4424d991014c0bf2f6a1521cbeb69659';
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
    # The state of the arm behavior
    
    # Header with timestamp
    std_msgs/Header header
    
    # Tee current state
    int8 state                         # Current state
    int8 INITIALIZING        = 0       # Waiting on child services, actions, etc.
    int8 UNKNOWN             = 1       # Waiting on feedback from driver
    int8 STOWED              = 2       # The arm is stowed
    int8 DEPLOYED            = 3       # The arm is deployed
    int8 SETTING             = 4       # The gripper is being set to a value
    int8 PANNING             = 5       # We are panning as part of a move
    int8 TILTING             = 6       # We are tilting as part of a move
    int8 STOWING_SETTING     = 7       # We are closing the gripper for stowing
    int8 STOWING_PANNING     = 8       # We are panning to zero for stowing
    int8 STOWING_TILTING     = 9       # We are tilting to zero for stowing
    int8 DEPLOYING_PANNING   = 10      # We are panning to zero for stowing
    int8 DEPLOYING_TILTING   = 11      # We are tilting to zero for stowing
    int8 CALIBRATING         = 12      # We are calibrating the gripper
    
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
    const resolved = new ArmState(null);
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
ArmState.Constants = {
  INITIALIZING: 0,
  UNKNOWN: 1,
  STOWED: 2,
  DEPLOYED: 3,
  SETTING: 4,
  PANNING: 5,
  TILTING: 6,
  STOWING_SETTING: 7,
  STOWING_PANNING: 8,
  STOWING_TILTING: 9,
  DEPLOYING_PANNING: 10,
  DEPLOYING_TILTING: 11,
  CALIBRATING: 12,
}

module.exports = ArmState;
