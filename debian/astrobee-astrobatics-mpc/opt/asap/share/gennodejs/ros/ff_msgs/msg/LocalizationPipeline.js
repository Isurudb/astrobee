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

class LocalizationPipeline {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.mode = null;
      this.name = null;
      this.requires_filter = null;
      this.requires_optical_flow = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = '';
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('requires_filter')) {
        this.requires_filter = initObj.requires_filter
      }
      else {
        this.requires_filter = false;
      }
      if (initObj.hasOwnProperty('requires_optical_flow')) {
        this.requires_optical_flow = initObj.requires_optical_flow
      }
      else {
        this.requires_optical_flow = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LocalizationPipeline
    // Serialize message field [id]
    bufferOffset = _serializer.string(obj.id, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.uint8(obj.mode, buffer, bufferOffset);
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [requires_filter]
    bufferOffset = _serializer.bool(obj.requires_filter, buffer, bufferOffset);
    // Serialize message field [requires_optical_flow]
    bufferOffset = _serializer.bool(obj.requires_optical_flow, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LocalizationPipeline
    let len;
    let data = new LocalizationPipeline(null);
    // Deserialize message field [id]
    data.id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [requires_filter]
    data.requires_filter = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [requires_optical_flow]
    data.requires_optical_flow = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.id.length;
    length += object.name.length;
    return length + 11;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/LocalizationPipeline';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '19b78d22f6e82148b4ff1aec54ea7e06';
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
    # Information about a pipeline
    
    string id                     # Short id for the pipeline
    uint8 mode                    # EKF mode for the pipeline
    string name                   # Long name for the pipe
    bool requires_filter          # Does this pipeline require the EKF
    bool requires_optical_flow    # Does this pipeline require optical flow
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LocalizationPipeline(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = ''
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.requires_filter !== undefined) {
      resolved.requires_filter = msg.requires_filter;
    }
    else {
      resolved.requires_filter = false
    }

    if (msg.requires_optical_flow !== undefined) {
      resolved.requires_optical_flow = msg.requires_optical_flow;
    }
    else {
      resolved.requires_optical_flow = false
    }

    return resolved;
    }
};

module.exports = LocalizationPipeline;
