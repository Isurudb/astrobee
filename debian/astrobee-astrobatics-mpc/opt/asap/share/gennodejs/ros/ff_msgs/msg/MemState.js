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

class MemState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.virt = null;
      this.virt_peak = null;
      this.ram = null;
      this.ram_peak = null;
      this.ram_perc = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('virt')) {
        this.virt = initObj.virt
      }
      else {
        this.virt = 0;
      }
      if (initObj.hasOwnProperty('virt_peak')) {
        this.virt_peak = initObj.virt_peak
      }
      else {
        this.virt_peak = 0;
      }
      if (initObj.hasOwnProperty('ram')) {
        this.ram = initObj.ram
      }
      else {
        this.ram = 0;
      }
      if (initObj.hasOwnProperty('ram_peak')) {
        this.ram_peak = initObj.ram_peak
      }
      else {
        this.ram_peak = 0;
      }
      if (initObj.hasOwnProperty('ram_perc')) {
        this.ram_perc = initObj.ram_perc
      }
      else {
        this.ram_perc = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MemState
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [virt]
    bufferOffset = _serializer.uint32(obj.virt, buffer, bufferOffset);
    // Serialize message field [virt_peak]
    bufferOffset = _serializer.uint32(obj.virt_peak, buffer, bufferOffset);
    // Serialize message field [ram]
    bufferOffset = _serializer.uint32(obj.ram, buffer, bufferOffset);
    // Serialize message field [ram_peak]
    bufferOffset = _serializer.uint32(obj.ram_peak, buffer, bufferOffset);
    // Serialize message field [ram_perc]
    bufferOffset = _serializer.float32(obj.ram_perc, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MemState
    let len;
    let data = new MemState(null);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [virt]
    data.virt = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [virt_peak]
    data.virt_peak = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [ram]
    data.ram = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [ram_peak]
    data.ram_peak = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [ram_perc]
    data.ram_perc = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.name.length;
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/MemState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '35fa33fe0824ebd7cf296b7a82e3c26b';
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
    # State of the Memory.
    
    # The memory load of the node, for the fields given in
    string name
    # Virtual Memory
    uint32 virt        # virtual memeory used in Mb
    uint32 virt_peak   # peak virtual memory used in Mb
    
    # Physical Memory
    uint32 ram        # physical memory used in Mb
    uint32 ram_peak   # peak physical memory used in Mb
    float32 ram_perc  # percentage of physical memory in %
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MemState(null);
    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.virt !== undefined) {
      resolved.virt = msg.virt;
    }
    else {
      resolved.virt = 0
    }

    if (msg.virt_peak !== undefined) {
      resolved.virt_peak = msg.virt_peak;
    }
    else {
      resolved.virt_peak = 0
    }

    if (msg.ram !== undefined) {
      resolved.ram = msg.ram;
    }
    else {
      resolved.ram = 0
    }

    if (msg.ram_peak !== undefined) {
      resolved.ram_peak = msg.ram_peak;
    }
    else {
      resolved.ram_peak = 0
    }

    if (msg.ram_perc !== undefined) {
      resolved.ram_perc = msg.ram_perc;
    }
    else {
      resolved.ram_perc = 0.0
    }

    return resolved;
    }
};

module.exports = MemState;
