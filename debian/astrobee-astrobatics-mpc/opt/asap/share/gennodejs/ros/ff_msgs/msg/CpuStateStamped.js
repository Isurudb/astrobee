// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let CpuState = require('./CpuState.js');
let CpuNodeState = require('./CpuNodeState.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class CpuStateStamped {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.name = null;
      this.load_fields = null;
      this.avg_loads = null;
      this.temp = null;
      this.cpus = null;
      this.load_nodes = null;
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
      if (initObj.hasOwnProperty('load_fields')) {
        this.load_fields = initObj.load_fields
      }
      else {
        this.load_fields = [];
      }
      if (initObj.hasOwnProperty('avg_loads')) {
        this.avg_loads = initObj.avg_loads
      }
      else {
        this.avg_loads = [];
      }
      if (initObj.hasOwnProperty('temp')) {
        this.temp = initObj.temp
      }
      else {
        this.temp = 0.0;
      }
      if (initObj.hasOwnProperty('cpus')) {
        this.cpus = initObj.cpus
      }
      else {
        this.cpus = [];
      }
      if (initObj.hasOwnProperty('load_nodes')) {
        this.load_nodes = initObj.load_nodes
      }
      else {
        this.load_nodes = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CpuStateStamped
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [load_fields]
    bufferOffset = _arraySerializer.string(obj.load_fields, buffer, bufferOffset, null);
    // Serialize message field [avg_loads]
    bufferOffset = _arraySerializer.float32(obj.avg_loads, buffer, bufferOffset, null);
    // Serialize message field [temp]
    bufferOffset = _serializer.float32(obj.temp, buffer, bufferOffset);
    // Serialize message field [cpus]
    // Serialize the length for message field [cpus]
    bufferOffset = _serializer.uint32(obj.cpus.length, buffer, bufferOffset);
    obj.cpus.forEach((val) => {
      bufferOffset = CpuState.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [load_nodes]
    // Serialize the length for message field [load_nodes]
    bufferOffset = _serializer.uint32(obj.load_nodes.length, buffer, bufferOffset);
    obj.load_nodes.forEach((val) => {
      bufferOffset = CpuNodeState.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CpuStateStamped
    let len;
    let data = new CpuStateStamped(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [load_fields]
    data.load_fields = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [avg_loads]
    data.avg_loads = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [temp]
    data.temp = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cpus]
    // Deserialize array length for message field [cpus]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.cpus = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.cpus[i] = CpuState.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [load_nodes]
    // Deserialize array length for message field [load_nodes]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.load_nodes = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.load_nodes[i] = CpuNodeState.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.name.length;
    object.load_fields.forEach((val) => {
      length += 4 + val.length;
    });
    length += 4 * object.avg_loads.length;
    object.cpus.forEach((val) => {
      length += CpuState.getMessageSize(val);
    });
    object.load_nodes.forEach((val) => {
      length += CpuNodeState.getMessageSize(val);
    });
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/CpuStateStamped';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ecff381c2d5b2d38dda690920e947e86';
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
    # Cpu state message with timestamp.
    
    # Header with timestamp
    std_msgs/Header header
    
    # Machine name (llp, hlp, mlp, etc)
    string name
    
    # Load constants
    string NICE=nice
    string USER=user
    string SYS=sys
    string VIRT=virt
    string TOTAL=total
    
    # The available fields within the load values, mostly uses the constants
    # defined above.
    string[] load_fields
    
    # Average loads for all processors combined
    float32[] avg_loads
    
    # Temperature for a cpu (average of all thermal zones)
    float32 temp
    
    # Information for each processor
    # Size of the array specifies how many processors are on the board, whether
    # or not all of them are enabled.
    ff_msgs/CpuState[] cpus
    
    # Load usage of individual ROS nodes
    ff_msgs/CpuNodeState[] load_nodes
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
    MSG: ff_msgs/CpuState
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
    
    ================================================================================
    MSG: ff_msgs/CpuNodeState
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
    # State of a CPU load for a node.
    
    # Node name
    string name
    
    # The load (in percentages) of the cpu
    float32 load 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CpuStateStamped(null);
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

    if (msg.load_fields !== undefined) {
      resolved.load_fields = msg.load_fields;
    }
    else {
      resolved.load_fields = []
    }

    if (msg.avg_loads !== undefined) {
      resolved.avg_loads = msg.avg_loads;
    }
    else {
      resolved.avg_loads = []
    }

    if (msg.temp !== undefined) {
      resolved.temp = msg.temp;
    }
    else {
      resolved.temp = 0.0
    }

    if (msg.cpus !== undefined) {
      resolved.cpus = new Array(msg.cpus.length);
      for (let i = 0; i < resolved.cpus.length; ++i) {
        resolved.cpus[i] = CpuState.Resolve(msg.cpus[i]);
      }
    }
    else {
      resolved.cpus = []
    }

    if (msg.load_nodes !== undefined) {
      resolved.load_nodes = new Array(msg.load_nodes.length);
      for (let i = 0; i < resolved.load_nodes.length; ++i) {
        resolved.load_nodes[i] = CpuNodeState.Resolve(msg.load_nodes[i]);
      }
    }
    else {
      resolved.load_nodes = []
    }

    return resolved;
    }
};

// Constants for message
CpuStateStamped.Constants = {
  NICE: 'nice',
  USER: 'user',
  SYS: 'sys',
  VIRT: 'virt',
  TOTAL: 'total',
}

module.exports = CpuStateStamped;
