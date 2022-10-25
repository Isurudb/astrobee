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

class CpuState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.enabled = null;
      this.loads = null;
      this.frequency = null;
      this.max_frequency = null;
    }
    else {
      if (initObj.hasOwnProperty('enabled')) {
        this.enabled = initObj.enabled
      }
      else {
        this.enabled = false;
      }
      if (initObj.hasOwnProperty('loads')) {
        this.loads = initObj.loads
      }
      else {
        this.loads = [];
      }
      if (initObj.hasOwnProperty('frequency')) {
        this.frequency = initObj.frequency
      }
      else {
        this.frequency = 0;
      }
      if (initObj.hasOwnProperty('max_frequency')) {
        this.max_frequency = initObj.max_frequency
      }
      else {
        this.max_frequency = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CpuState
    // Serialize message field [enabled]
    bufferOffset = _serializer.bool(obj.enabled, buffer, bufferOffset);
    // Serialize message field [loads]
    bufferOffset = _arraySerializer.float32(obj.loads, buffer, bufferOffset, null);
    // Serialize message field [frequency]
    bufferOffset = _serializer.uint32(obj.frequency, buffer, bufferOffset);
    // Serialize message field [max_frequency]
    bufferOffset = _serializer.uint32(obj.max_frequency, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CpuState
    let len;
    let data = new CpuState(null);
    // Deserialize message field [enabled]
    data.enabled = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [loads]
    data.loads = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [frequency]
    data.frequency = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [max_frequency]
    data.max_frequency = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.loads.length;
    return length + 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/CpuState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3ff6c0a8b78ea1e9087461c1e42b6ca2';
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
    # State of a CPU.
    
    # Processor is on (enabled) or not
    bool enabled
    
    # The load (in percentages) of the cpu, for the fields given in
    # CpuStateStamped
    float32[] loads 
    
    # Current operating frequency in Hz
    uint32 frequency
    
    # Max frequency (may be less than theoretical limit of the processor)
    uint32 max_frequency
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CpuState(null);
    if (msg.enabled !== undefined) {
      resolved.enabled = msg.enabled;
    }
    else {
      resolved.enabled = false
    }

    if (msg.loads !== undefined) {
      resolved.loads = msg.loads;
    }
    else {
      resolved.loads = []
    }

    if (msg.frequency !== undefined) {
      resolved.frequency = msg.frequency;
    }
    else {
      resolved.frequency = 0
    }

    if (msg.max_frequency !== undefined) {
      resolved.max_frequency = msg.max_frequency;
    }
    else {
      resolved.max_frequency = 0
    }

    return resolved;
    }
};

module.exports = CpuState;
