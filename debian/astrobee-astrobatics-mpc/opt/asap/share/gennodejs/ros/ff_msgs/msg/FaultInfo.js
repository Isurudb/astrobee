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

class FaultInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.subsystem = null;
      this.node = null;
      this.id = null;
      this.warning = null;
      this.description = null;
    }
    else {
      if (initObj.hasOwnProperty('subsystem')) {
        this.subsystem = initObj.subsystem
      }
      else {
        this.subsystem = 0;
      }
      if (initObj.hasOwnProperty('node')) {
        this.node = initObj.node
      }
      else {
        this.node = 0;
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('warning')) {
        this.warning = initObj.warning
      }
      else {
        this.warning = false;
      }
      if (initObj.hasOwnProperty('description')) {
        this.description = initObj.description
      }
      else {
        this.description = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FaultInfo
    // Serialize message field [subsystem]
    bufferOffset = _serializer.uint16(obj.subsystem, buffer, bufferOffset);
    // Serialize message field [node]
    bufferOffset = _serializer.uint16(obj.node, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.uint32(obj.id, buffer, bufferOffset);
    // Serialize message field [warning]
    bufferOffset = _serializer.bool(obj.warning, buffer, bufferOffset);
    // Serialize message field [description]
    bufferOffset = _serializer.string(obj.description, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FaultInfo
    let len;
    let data = new FaultInfo(null);
    // Deserialize message field [subsystem]
    data.subsystem = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [node]
    data.node = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [warning]
    data.warning = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [description]
    data.description = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.description.length;
    return length + 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/FaultInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1f6014a9106a0f40b77f475f6f9592fa';
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
    # Fault info message is used in the fault config message to contain all the 
    # information GDS needs to know about a fault
    
    uint16 subsystem    # index into subsystem names array found in fault config msg
    
    uint16 node         # index into node names array found in fault config msg
    
    uint32 id           # id corresponding to the fault
    
    bool warning        # whether the fault is a warning or not
    
    string description  # A short description of why the fault occurred
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FaultInfo(null);
    if (msg.subsystem !== undefined) {
      resolved.subsystem = msg.subsystem;
    }
    else {
      resolved.subsystem = 0
    }

    if (msg.node !== undefined) {
      resolved.node = msg.node;
    }
    else {
      resolved.node = 0
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.warning !== undefined) {
      resolved.warning = msg.warning;
    }
    else {
      resolved.warning = false
    }

    if (msg.description !== undefined) {
      resolved.description = msg.description;
    }
    else {
      resolved.description = ''
    }

    return resolved;
    }
};

module.exports = FaultInfo;
