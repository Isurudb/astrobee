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

class FaultData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.key = null;
      this.data_type = null;
      this.f = null;
      this.i = null;
      this.s = null;
    }
    else {
      if (initObj.hasOwnProperty('key')) {
        this.key = initObj.key
      }
      else {
        this.key = '';
      }
      if (initObj.hasOwnProperty('data_type')) {
        this.data_type = initObj.data_type
      }
      else {
        this.data_type = 0;
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
      if (initObj.hasOwnProperty('s')) {
        this.s = initObj.s
      }
      else {
        this.s = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FaultData
    // Serialize message field [key]
    bufferOffset = _serializer.string(obj.key, buffer, bufferOffset);
    // Serialize message field [data_type]
    bufferOffset = _serializer.uint8(obj.data_type, buffer, bufferOffset);
    // Serialize message field [f]
    bufferOffset = _serializer.float32(obj.f, buffer, bufferOffset);
    // Serialize message field [i]
    bufferOffset = _serializer.int32(obj.i, buffer, bufferOffset);
    // Serialize message field [s]
    bufferOffset = _serializer.string(obj.s, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FaultData
    let len;
    let data = new FaultData(null);
    // Deserialize message field [key]
    data.key = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [data_type]
    data.data_type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [f]
    data.f = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [i]
    data.i = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [s]
    data.s = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.key.length;
    length += object.s.length;
    return length + 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/FaultData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '632c6de83aa53364cbd36514ffa5c853';
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
    # Fault data messsage contains information of why the fault occurred
    
    uint8 DATA_TYPE_FLOAT   = 0   # Data in this msg is of type float
    uint8 DATA_TYPE_INT     = 1   # Data in this msg is of type int
    uint8 DATA_TYPE_STRING  = 2   # Data in this msg is of type string
    
    string key  # Specifies what the data in the msg is, can only be 32 chars long
    
    uint8 data_type   # Specifies the type of data in the message
    
    float32 f   # Value used for fault analysis, data_type must be 0
    int32 i     # Value used for fault analysis, data_type must be 1
    string s    # String used for fault analysis, data_type must be 2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FaultData(null);
    if (msg.key !== undefined) {
      resolved.key = msg.key;
    }
    else {
      resolved.key = ''
    }

    if (msg.data_type !== undefined) {
      resolved.data_type = msg.data_type;
    }
    else {
      resolved.data_type = 0
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

    if (msg.s !== undefined) {
      resolved.s = msg.s;
    }
    else {
      resolved.s = ''
    }

    return resolved;
    }
};

// Constants for message
FaultData.Constants = {
  DATA_TYPE_FLOAT: 0,
  DATA_TYPE_INT: 1,
  DATA_TYPE_STRING: 2,
}

module.exports = FaultData;
