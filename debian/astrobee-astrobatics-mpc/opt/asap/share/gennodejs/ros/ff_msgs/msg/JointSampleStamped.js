// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let JointSample = require('./JointSample.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class JointSampleStamped {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.samples = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('samples')) {
        this.samples = initObj.samples
      }
      else {
        this.samples = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JointSampleStamped
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [samples]
    // Serialize the length for message field [samples]
    bufferOffset = _serializer.uint32(obj.samples.length, buffer, bufferOffset);
    obj.samples.forEach((val) => {
      bufferOffset = JointSample.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JointSampleStamped
    let len;
    let data = new JointSampleStamped(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [samples]
    // Deserialize array length for message field [samples]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.samples = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.samples[i] = JointSample.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.samples.forEach((val) => {
      length += JointSample.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/JointSampleStamped';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ada8b7552d00a44b3992523c84bda644';
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
    # An array of Joint sample messages.
    
    # Header with timestamp
    std_msgs/Header header
    
    # Joint samples
    ff_msgs/JointSample[] samples
    
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
    MSG: ff_msgs/JointSample
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
    # JointSample message, based off of rapid::JointSample
    
    # Flag values for joint status. 
    # Joint is enabled
    uint8 JOINT_ENABLED    = 0      # Joint enabled
    uint8 JOINT_DISABLED   = 1      # Joint disabled
    
    
    # Angle position (in radians) of the joint
    float32 angle_pos
    
    # Angle velocity (in radians/sec) of the joint
    float32 angle_vel
    
    # Angle acceleration (in radians/sec^2) of the joint (not being used)
    float32 angle_acc
    
    # Current draw of joint motor
    float32 current
    
    # Torque sensed at the joint (not being used)
    float32 torque
    
    # Temperature of the joint (in Celsius)
    float32 temperature
    
    # Bit field representing the state of the joint
    uint16 status
    
    # Human-readable name
    string name
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JointSampleStamped(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.samples !== undefined) {
      resolved.samples = new Array(msg.samples.length);
      for (let i = 0; i < resolved.samples.length; ++i) {
        resolved.samples[i] = JointSample.Resolve(msg.samples[i]);
      }
    }
    else {
      resolved.samples = []
    }

    return resolved;
    }
};

module.exports = JointSampleStamped;
