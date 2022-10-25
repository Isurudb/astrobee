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

class SignalState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.state = null;
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
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SignalState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.uint8(obj.state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SignalState
    let len;
    let data = new SignalState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 1;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/SignalState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1d27ab072e4b4bf58f04e75ff6768d4e';
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
    # Signal state which is based on what the Astrobee is doing. Should be used to
    # figure out what should be displayed on the signal lights and touch screen.
    
    # Header with timestamp
    std_msgs/Header header
    
    uint8 VIDEO_ON              = 0
    uint8 VIDEO_OFF             = 1
    uint8 SUCCESS               = 3
    uint8 ENTER_HATCHWAY        = 4
    uint8 UNDOCK                = 5
    uint8 UNPERCH               = 6
    uint8 MOTION_IMPAIRED       = 7
    uint8 THRUST_FORWARD        = 8
    uint8 THRUST_AFT            = 9
    uint8 TURN_RIGHT            = 10
    uint8 TURN_LEFT             = 11
    uint8 TURN_UP               = 12
    uint8 TURN_DOWN             = 13
    uint8 CLEAR                 = 14
    uint8 SLEEP                 = 15
    uint8 WAKE                  = 16
    uint8 STOP_ALL_LIGHTS       = 17
    uint8 CHARGING              = 18
    
    # Signal state
    uint8 state
    
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
    const resolved = new SignalState(null);
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

    return resolved;
    }
};

// Constants for message
SignalState.Constants = {
  VIDEO_ON: 0,
  VIDEO_OFF: 1,
  SUCCESS: 3,
  ENTER_HATCHWAY: 4,
  UNDOCK: 5,
  UNPERCH: 6,
  MOTION_IMPAIRED: 7,
  THRUST_FORWARD: 8,
  THRUST_AFT: 9,
  TURN_RIGHT: 10,
  TURN_LEFT: 11,
  TURN_UP: 12,
  TURN_DOWN: 13,
  CLEAR: 14,
  SLEEP: 15,
  WAKE: 16,
  STOP_ALL_LIGHTS: 17,
  CHARGING: 18,
}

module.exports = SignalState;
