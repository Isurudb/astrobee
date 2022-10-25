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

class Performance {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.stamp = null;
      this.count = null;
      this.last = null;
      this.min = null;
      this.max = null;
      this.mean = null;
      this.stddev = null;
      this.var = null;
    }
    else {
      if (initObj.hasOwnProperty('stamp')) {
        this.stamp = initObj.stamp
      }
      else {
        this.stamp = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('count')) {
        this.count = initObj.count
      }
      else {
        this.count = 0.0;
      }
      if (initObj.hasOwnProperty('last')) {
        this.last = initObj.last
      }
      else {
        this.last = 0.0;
      }
      if (initObj.hasOwnProperty('min')) {
        this.min = initObj.min
      }
      else {
        this.min = 0.0;
      }
      if (initObj.hasOwnProperty('max')) {
        this.max = initObj.max
      }
      else {
        this.max = 0.0;
      }
      if (initObj.hasOwnProperty('mean')) {
        this.mean = initObj.mean
      }
      else {
        this.mean = 0.0;
      }
      if (initObj.hasOwnProperty('stddev')) {
        this.stddev = initObj.stddev
      }
      else {
        this.stddev = 0.0;
      }
      if (initObj.hasOwnProperty('var')) {
        this.var = initObj.var
      }
      else {
        this.var = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Performance
    // Serialize message field [stamp]
    bufferOffset = _serializer.time(obj.stamp, buffer, bufferOffset);
    // Serialize message field [count]
    bufferOffset = _serializer.float32(obj.count, buffer, bufferOffset);
    // Serialize message field [last]
    bufferOffset = _serializer.float32(obj.last, buffer, bufferOffset);
    // Serialize message field [min]
    bufferOffset = _serializer.float32(obj.min, buffer, bufferOffset);
    // Serialize message field [max]
    bufferOffset = _serializer.float32(obj.max, buffer, bufferOffset);
    // Serialize message field [mean]
    bufferOffset = _serializer.float32(obj.mean, buffer, bufferOffset);
    // Serialize message field [stddev]
    bufferOffset = _serializer.float32(obj.stddev, buffer, bufferOffset);
    // Serialize message field [var]
    bufferOffset = _serializer.float32(obj.var, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Performance
    let len;
    let data = new Performance(null);
    // Deserialize message field [stamp]
    data.stamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [count]
    data.count = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [last]
    data.last = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [min]
    data.min = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [max]
    data.max = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [mean]
    data.mean = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [stddev]
    data.stddev = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [var]
    data.var = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 36;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/Performance';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '04ada46adc18b617f396f9d156029c85';
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
    # Statistics used to measure performance.
    
    time stamp
    float32 count
    float32 last
    float32 min
    float32 max
    float32 mean
    float32 stddev
    float32 var
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Performance(null);
    if (msg.stamp !== undefined) {
      resolved.stamp = msg.stamp;
    }
    else {
      resolved.stamp = {secs: 0, nsecs: 0}
    }

    if (msg.count !== undefined) {
      resolved.count = msg.count;
    }
    else {
      resolved.count = 0.0
    }

    if (msg.last !== undefined) {
      resolved.last = msg.last;
    }
    else {
      resolved.last = 0.0
    }

    if (msg.min !== undefined) {
      resolved.min = msg.min;
    }
    else {
      resolved.min = 0.0
    }

    if (msg.max !== undefined) {
      resolved.max = msg.max;
    }
    else {
      resolved.max = 0.0
    }

    if (msg.mean !== undefined) {
      resolved.mean = msg.mean;
    }
    else {
      resolved.mean = 0.0
    }

    if (msg.stddev !== undefined) {
      resolved.stddev = msg.stddev;
    }
    else {
      resolved.stddev = 0.0
    }

    if (msg.var !== undefined) {
      resolved.var = msg.var;
    }
    else {
      resolved.var = 0.0
    }

    return resolved;
    }
};

module.exports = Performance;
