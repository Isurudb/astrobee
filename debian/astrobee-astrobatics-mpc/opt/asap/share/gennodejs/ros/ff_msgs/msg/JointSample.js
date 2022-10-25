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

class JointSample {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.angle_pos = null;
      this.angle_vel = null;
      this.angle_acc = null;
      this.current = null;
      this.torque = null;
      this.temperature = null;
      this.status = null;
      this.name = null;
    }
    else {
      if (initObj.hasOwnProperty('angle_pos')) {
        this.angle_pos = initObj.angle_pos
      }
      else {
        this.angle_pos = 0.0;
      }
      if (initObj.hasOwnProperty('angle_vel')) {
        this.angle_vel = initObj.angle_vel
      }
      else {
        this.angle_vel = 0.0;
      }
      if (initObj.hasOwnProperty('angle_acc')) {
        this.angle_acc = initObj.angle_acc
      }
      else {
        this.angle_acc = 0.0;
      }
      if (initObj.hasOwnProperty('current')) {
        this.current = initObj.current
      }
      else {
        this.current = 0.0;
      }
      if (initObj.hasOwnProperty('torque')) {
        this.torque = initObj.torque
      }
      else {
        this.torque = 0.0;
      }
      if (initObj.hasOwnProperty('temperature')) {
        this.temperature = initObj.temperature
      }
      else {
        this.temperature = 0.0;
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = 0;
      }
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JointSample
    // Serialize message field [angle_pos]
    bufferOffset = _serializer.float32(obj.angle_pos, buffer, bufferOffset);
    // Serialize message field [angle_vel]
    bufferOffset = _serializer.float32(obj.angle_vel, buffer, bufferOffset);
    // Serialize message field [angle_acc]
    bufferOffset = _serializer.float32(obj.angle_acc, buffer, bufferOffset);
    // Serialize message field [current]
    bufferOffset = _serializer.float32(obj.current, buffer, bufferOffset);
    // Serialize message field [torque]
    bufferOffset = _serializer.float32(obj.torque, buffer, bufferOffset);
    // Serialize message field [temperature]
    bufferOffset = _serializer.float32(obj.temperature, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.uint16(obj.status, buffer, bufferOffset);
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JointSample
    let len;
    let data = new JointSample(null);
    // Deserialize message field [angle_pos]
    data.angle_pos = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angle_vel]
    data.angle_vel = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angle_acc]
    data.angle_acc = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current]
    data.current = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [torque]
    data.torque = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [temperature]
    data.temperature = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.name.length;
    return length + 30;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/JointSample';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fe238686c8b329629bd0aa9499404e2e';
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
    const resolved = new JointSample(null);
    if (msg.angle_pos !== undefined) {
      resolved.angle_pos = msg.angle_pos;
    }
    else {
      resolved.angle_pos = 0.0
    }

    if (msg.angle_vel !== undefined) {
      resolved.angle_vel = msg.angle_vel;
    }
    else {
      resolved.angle_vel = 0.0
    }

    if (msg.angle_acc !== undefined) {
      resolved.angle_acc = msg.angle_acc;
    }
    else {
      resolved.angle_acc = 0.0
    }

    if (msg.current !== undefined) {
      resolved.current = msg.current;
    }
    else {
      resolved.current = 0.0
    }

    if (msg.torque !== undefined) {
      resolved.torque = msg.torque;
    }
    else {
      resolved.torque = 0.0
    }

    if (msg.temperature !== undefined) {
      resolved.temperature = msg.temperature;
    }
    else {
      resolved.temperature = 0.0
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = 0
    }

    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    return resolved;
    }
};

// Constants for message
JointSample.Constants = {
  JOINT_ENABLED: 0,
  JOINT_DISABLED: 1,
}

module.exports = JointSample;
