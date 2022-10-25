// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let CommandArg = require('./CommandArg.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class CommandStamped {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.cmd_name = null;
      this.cmd_id = null;
      this.cmd_src = null;
      this.cmd_origin = null;
      this.subsys_name = null;
      this.args = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('cmd_name')) {
        this.cmd_name = initObj.cmd_name
      }
      else {
        this.cmd_name = '';
      }
      if (initObj.hasOwnProperty('cmd_id')) {
        this.cmd_id = initObj.cmd_id
      }
      else {
        this.cmd_id = '';
      }
      if (initObj.hasOwnProperty('cmd_src')) {
        this.cmd_src = initObj.cmd_src
      }
      else {
        this.cmd_src = '';
      }
      if (initObj.hasOwnProperty('cmd_origin')) {
        this.cmd_origin = initObj.cmd_origin
      }
      else {
        this.cmd_origin = '';
      }
      if (initObj.hasOwnProperty('subsys_name')) {
        this.subsys_name = initObj.subsys_name
      }
      else {
        this.subsys_name = '';
      }
      if (initObj.hasOwnProperty('args')) {
        this.args = initObj.args
      }
      else {
        this.args = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CommandStamped
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [cmd_name]
    bufferOffset = _serializer.string(obj.cmd_name, buffer, bufferOffset);
    // Serialize message field [cmd_id]
    bufferOffset = _serializer.string(obj.cmd_id, buffer, bufferOffset);
    // Serialize message field [cmd_src]
    bufferOffset = _serializer.string(obj.cmd_src, buffer, bufferOffset);
    // Serialize message field [cmd_origin]
    bufferOffset = _serializer.string(obj.cmd_origin, buffer, bufferOffset);
    // Serialize message field [subsys_name]
    bufferOffset = _serializer.string(obj.subsys_name, buffer, bufferOffset);
    // Serialize message field [args]
    // Serialize the length for message field [args]
    bufferOffset = _serializer.uint32(obj.args.length, buffer, bufferOffset);
    obj.args.forEach((val) => {
      bufferOffset = CommandArg.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CommandStamped
    let len;
    let data = new CommandStamped(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [cmd_name]
    data.cmd_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [cmd_id]
    data.cmd_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [cmd_src]
    data.cmd_src = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [cmd_origin]
    data.cmd_origin = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [subsys_name]
    data.subsys_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [args]
    // Deserialize array length for message field [args]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.args = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.args[i] = CommandArg.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.cmd_name.length;
    length += object.cmd_id.length;
    length += object.cmd_src.length;
    length += object.cmd_origin.length;
    length += object.subsys_name.length;
    object.args.forEach((val) => {
      length += CommandArg.getMessageSize(val);
    });
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/CommandStamped';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ac01350894b1be9e3a7b4f390a14812d';
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
    # Command Message, loosely based off of the RAPID Command.idl
    
    # Header with timestamp
    std_msgs/Header header
    
    # Command name
    string cmd_name
    
    # Unique identifier for command = unique counter + participant + timestamp
    string cmd_id
    
    # Source of the command, either operators, the system monitor or guest science
    string cmd_src
    
    # Origin of the command, ground for operators, astrobee for another astrobee,
    # sys_monitor for fault responses, and guest_science for guest science
    # commands
    string cmd_origin
    
    # Name of subsystem the command is going to (not used but kept to be consistant
    # with the command idl)
    string subsys_name
    
    # Arguments for the command 
    ff_msgs/CommandArg[] args
    
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
    MSG: ff_msgs/CommandArg
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
    # An argument to a command sent through RAPID
    #
    # Note that this is approximating a union in DDS. However, this is an
    # inefficient union, and thus each instance will take up at least 89 bytes.
    # However, even with the maximum of 16 arguments to a command, we only have
    # about 1k extra data. I, tfmorse, am ok with that. Commands are rarely sent.
    
    uint8 DATA_TYPE_BOOL     = 0
    uint8 DATA_TYPE_DOUBLE   = 1
    uint8 DATA_TYPE_FLOAT    = 2
    uint8 DATA_TYPE_INT      = 3
    uint8 DATA_TYPE_LONGLONG = 4
    uint8 DATA_TYPE_STRING   = 5
    uint8 DATA_TYPE_VEC3d    = 6
    uint8 DATA_TYPE_MAT33f   = 7
    
    uint8 data_type
    
    bool b
    float64 d
    float32 f
    int32 i
    int64 ll
    string s
    float64[3] vec3d
    float32[9] mat33f
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CommandStamped(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.cmd_name !== undefined) {
      resolved.cmd_name = msg.cmd_name;
    }
    else {
      resolved.cmd_name = ''
    }

    if (msg.cmd_id !== undefined) {
      resolved.cmd_id = msg.cmd_id;
    }
    else {
      resolved.cmd_id = ''
    }

    if (msg.cmd_src !== undefined) {
      resolved.cmd_src = msg.cmd_src;
    }
    else {
      resolved.cmd_src = ''
    }

    if (msg.cmd_origin !== undefined) {
      resolved.cmd_origin = msg.cmd_origin;
    }
    else {
      resolved.cmd_origin = ''
    }

    if (msg.subsys_name !== undefined) {
      resolved.subsys_name = msg.subsys_name;
    }
    else {
      resolved.subsys_name = ''
    }

    if (msg.args !== undefined) {
      resolved.args = new Array(msg.args.length);
      for (let i = 0; i < resolved.args.length; ++i) {
        resolved.args[i] = CommandArg.Resolve(msg.args[i]);
      }
    }
    else {
      resolved.args = []
    }

    return resolved;
    }
};

module.exports = CommandStamped;
