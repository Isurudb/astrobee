// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let MemState = require('./MemState.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class MemStateStamped {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.name = null;
      this.ram_total = null;
      this.ram_used = null;
      this.virt_total = null;
      this.virt_used = null;
      this.nodes = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('ram_total')) {
        this.ram_total = initObj.ram_total
      }
      else {
        this.ram_total = 0;
      }
      if (initObj.hasOwnProperty('ram_used')) {
        this.ram_used = initObj.ram_used
      }
      else {
        this.ram_used = 0;
      }
      if (initObj.hasOwnProperty('virt_total')) {
        this.virt_total = initObj.virt_total
      }
      else {
        this.virt_total = 0;
      }
      if (initObj.hasOwnProperty('virt_used')) {
        this.virt_used = initObj.virt_used
      }
      else {
        this.virt_used = 0;
      }
      if (initObj.hasOwnProperty('nodes')) {
        this.nodes = initObj.nodes
      }
      else {
        this.nodes = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MemStateStamped
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [ram_total]
    bufferOffset = _serializer.uint32(obj.ram_total, buffer, bufferOffset);
    // Serialize message field [ram_used]
    bufferOffset = _serializer.uint32(obj.ram_used, buffer, bufferOffset);
    // Serialize message field [virt_total]
    bufferOffset = _serializer.uint32(obj.virt_total, buffer, bufferOffset);
    // Serialize message field [virt_used]
    bufferOffset = _serializer.uint32(obj.virt_used, buffer, bufferOffset);
    // Serialize message field [nodes]
    // Serialize the length for message field [nodes]
    bufferOffset = _serializer.uint32(obj.nodes.length, buffer, bufferOffset);
    obj.nodes.forEach((val) => {
      bufferOffset = MemState.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MemStateStamped
    let len;
    let data = new MemStateStamped(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [ram_total]
    data.ram_total = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [ram_used]
    data.ram_used = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [virt_total]
    data.virt_total = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [virt_used]
    data.virt_used = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [nodes]
    // Deserialize array length for message field [nodes]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.nodes = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.nodes[i] = MemState.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.name.length;
    object.nodes.forEach((val) => {
      length += MemState.getMessageSize(val);
    });
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/MemStateStamped';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '331992e4d9de11a301b654de12ecbac9';
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
    # Memory state message with timestamp.
    
    # Header with timestamp
    std_msgs/Header header
    
    # Machine name (llp, hlp, mlp, etc)
    string name
    
    # Physical Memory (RAM)
    uint32 ram_total   # total physical memeory in system Mb
    uint32 ram_used    # totalphysical memeory used in Mb
    
    # Virtual Memory
    uint32 virt_total  # total virtual memeory in system in Mb
    uint32 virt_used   # total virtual memeory used in Mb
    
    # Individual nodes
    ff_msgs/MemState[] nodes
    
    
    
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
    MSG: ff_msgs/MemState
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
    const resolved = new MemStateStamped(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.ram_total !== undefined) {
      resolved.ram_total = msg.ram_total;
    }
    else {
      resolved.ram_total = 0
    }

    if (msg.ram_used !== undefined) {
      resolved.ram_used = msg.ram_used;
    }
    else {
      resolved.ram_used = 0
    }

    if (msg.virt_total !== undefined) {
      resolved.virt_total = msg.virt_total;
    }
    else {
      resolved.virt_total = 0
    }

    if (msg.virt_used !== undefined) {
      resolved.virt_used = msg.virt_used;
    }
    else {
      resolved.virt_used = 0
    }

    if (msg.nodes !== undefined) {
      resolved.nodes = new Array(msg.nodes.length);
      for (let i = 0; i < resolved.nodes.length; ++i) {
        resolved.nodes[i] = MemState.Resolve(msg.nodes[i]);
      }
    }
    else {
      resolved.nodes = []
    }

    return resolved;
    }
};

module.exports = MemStateStamped;
