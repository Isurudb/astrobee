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

class CommandArg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.data_type = null;
      this.b = null;
      this.d = null;
      this.f = null;
      this.i = null;
      this.ll = null;
      this.s = null;
      this.vec3d = null;
      this.mat33f = null;
    }
    else {
      if (initObj.hasOwnProperty('data_type')) {
        this.data_type = initObj.data_type
      }
      else {
        this.data_type = 0;
      }
      if (initObj.hasOwnProperty('b')) {
        this.b = initObj.b
      }
      else {
        this.b = false;
      }
      if (initObj.hasOwnProperty('d')) {
        this.d = initObj.d
      }
      else {
        this.d = 0.0;
      }
      if (initObj.hasOwnProperty('f')) {
        this.f = initObj.f
      }
      else {
        this.f = 0.0;
      }
      if (initObj.hasOwnProperty('i')) {
        this.i = initObj.i
      }
      else {
        this.i = 0;
      }
      if (initObj.hasOwnProperty('ll')) {
        this.ll = initObj.ll
      }
      else {
        this.ll = 0;
      }
      if (initObj.hasOwnProperty('s')) {
        this.s = initObj.s
      }
      else {
        this.s = '';
      }
      if (initObj.hasOwnProperty('vec3d')) {
        this.vec3d = initObj.vec3d
      }
      else {
        this.vec3d = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('mat33f')) {
        this.mat33f = initObj.mat33f
      }
      else {
        this.mat33f = new Array(9).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CommandArg
    // Serialize message field [data_type]
    bufferOffset = _serializer.uint8(obj.data_type, buffer, bufferOffset);
    // Serialize message field [b]
    bufferOffset = _serializer.bool(obj.b, buffer, bufferOffset);
    // Serialize message field [d]
    bufferOffset = _serializer.float64(obj.d, buffer, bufferOffset);
    // Serialize message field [f]
    bufferOffset = _serializer.float32(obj.f, buffer, bufferOffset);
    // Serialize message field [i]
    bufferOffset = _serializer.int32(obj.i, buffer, bufferOffset);
    // Serialize message field [ll]
    bufferOffset = _serializer.int64(obj.ll, buffer, bufferOffset);
    // Serialize message field [s]
    bufferOffset = _serializer.string(obj.s, buffer, bufferOffset);
    // Check that the constant length array field [vec3d] has the right length
    if (obj.vec3d.length !== 3) {
      throw new Error('Unable to serialize array field vec3d - length must be 3')
    }
    // Serialize message field [vec3d]
    bufferOffset = _arraySerializer.float64(obj.vec3d, buffer, bufferOffset, 3);
    // Check that the constant length array field [mat33f] has the right length
    if (obj.mat33f.length !== 9) {
      throw new Error('Unable to serialize array field mat33f - length must be 9')
    }
    // Serialize message field [mat33f]
    bufferOffset = _arraySerializer.float32(obj.mat33f, buffer, bufferOffset, 9);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CommandArg
    let len;
    let data = new CommandArg(null);
    // Deserialize message field [data_type]
    data.data_type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [b]
    data.b = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [d]
    data.d = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [f]
    data.f = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [i]
    data.i = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ll]
    data.ll = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [s]
    data.s = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [vec3d]
    data.vec3d = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [mat33f]
    data.mat33f = _arrayDeserializer.float32(buffer, bufferOffset, 9)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.s.length;
    return length + 90;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/CommandArg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c64f399f685551792b2e185eb2878830';
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
    const resolved = new CommandArg(null);
    if (msg.data_type !== undefined) {
      resolved.data_type = msg.data_type;
    }
    else {
      resolved.data_type = 0
    }

    if (msg.b !== undefined) {
      resolved.b = msg.b;
    }
    else {
      resolved.b = false
    }

    if (msg.d !== undefined) {
      resolved.d = msg.d;
    }
    else {
      resolved.d = 0.0
    }

    if (msg.f !== undefined) {
      resolved.f = msg.f;
    }
    else {
      resolved.f = 0.0
    }

    if (msg.i !== undefined) {
      resolved.i = msg.i;
    }
    else {
      resolved.i = 0
    }

    if (msg.ll !== undefined) {
      resolved.ll = msg.ll;
    }
    else {
      resolved.ll = 0
    }

    if (msg.s !== undefined) {
      resolved.s = msg.s;
    }
    else {
      resolved.s = ''
    }

    if (msg.vec3d !== undefined) {
      resolved.vec3d = msg.vec3d;
    }
    else {
      resolved.vec3d = new Array(3).fill(0)
    }

    if (msg.mat33f !== undefined) {
      resolved.mat33f = msg.mat33f;
    }
    else {
      resolved.mat33f = new Array(9).fill(0)
    }

    return resolved;
    }
};

// Constants for message
CommandArg.Constants = {
  DATA_TYPE_BOOL: 0,
  DATA_TYPE_DOUBLE: 1,
  DATA_TYPE_FLOAT: 2,
  DATA_TYPE_INT: 3,
  DATA_TYPE_LONGLONG: 4,
  DATA_TYPE_STRING: 5,
  DATA_TYPE_VEC3D: 6,
  DATA_TYPE_MAT33F: 7,
}

module.exports = CommandArg;
