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

class MobilityState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.state = null;
      this.sub_state = null;
    }
    else {
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = 0;
      }
      if (initObj.hasOwnProperty('sub_state')) {
        this.sub_state = initObj.sub_state
      }
      else {
        this.sub_state = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MobilityState
    // Serialize message field [state]
    bufferOffset = _serializer.uint8(obj.state, buffer, bufferOffset);
    // Serialize message field [sub_state]
    bufferOffset = _serializer.int32(obj.sub_state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MobilityState
    let len;
    let data = new MobilityState(null);
    // Deserialize message field [state]
    data.state = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [sub_state]
    data.sub_state = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/MobilityState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2c5f9184aace6b4675fe28aa28d9047e';
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
    # Mobility states, based off the enumeration constants in
    # rapid::ext::astrobee::AgentState
    #
    # *MUST* be kept in sync with the DDS IDL file in astrobee_common
    
    uint8 DRIFTING        = 0   # Astrobee's propulsion is off
    uint8 STOPPING        = 1   # Astrobee is either stopping or stopped
    uint8 FLYING          = 2   # Astrobee is flying
    uint8 DOCKING         = 3   # Astrobee is either docking or undocking
    uint8 PERCHING        = 4   # Astrobee is either perching or unperching
    
    # Mobility state
    uint8 state
    
    # Specifies the progress of the action. For docking, this value can be N to -N
    # where N through 1 specifies the progress of a docking action, 0 is docked, and
    # -1 through -N specifies the process of an undocking action. For stopping, this
    # value is either 1 or 0 where 1 means the robot is coming to a stop and 0 means
    # the robot is stopped. For perching, this value can be N to -N where N through
    # 1 specifies the progress of a perching action, 0 is perched, and -1 through
    # -N specifies the process of an unperching action.
    int32 sub_state
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MobilityState(null);
    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = 0
    }

    if (msg.sub_state !== undefined) {
      resolved.sub_state = msg.sub_state;
    }
    else {
      resolved.sub_state = 0
    }

    return resolved;
    }
};

// Constants for message
MobilityState.Constants = {
  DRIFTING: 0,
  STOPPING: 1,
  FLYING: 2,
  DOCKING: 3,
  PERCHING: 4,
}

module.exports = MobilityState;
