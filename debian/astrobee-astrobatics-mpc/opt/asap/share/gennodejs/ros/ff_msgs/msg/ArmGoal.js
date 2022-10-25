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

class ArmGoal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.command = null;
      this.pan = null;
      this.tilt = null;
      this.gripper = null;
    }
    else {
      if (initObj.hasOwnProperty('command')) {
        this.command = initObj.command
      }
      else {
        this.command = 0;
      }
      if (initObj.hasOwnProperty('pan')) {
        this.pan = initObj.pan
      }
      else {
        this.pan = 0.0;
      }
      if (initObj.hasOwnProperty('tilt')) {
        this.tilt = initObj.tilt
      }
      else {
        this.tilt = 0.0;
      }
      if (initObj.hasOwnProperty('gripper')) {
        this.gripper = initObj.gripper
      }
      else {
        this.gripper = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ArmGoal
    // Serialize message field [command]
    bufferOffset = _serializer.uint8(obj.command, buffer, bufferOffset);
    // Serialize message field [pan]
    bufferOffset = _serializer.float32(obj.pan, buffer, bufferOffset);
    // Serialize message field [tilt]
    bufferOffset = _serializer.float32(obj.tilt, buffer, bufferOffset);
    // Serialize message field [gripper]
    bufferOffset = _serializer.float32(obj.gripper, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArmGoal
    let len;
    let data = new ArmGoal(null);
    // Deserialize message field [command]
    data.command = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [pan]
    data.pan = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tilt]
    data.tilt = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gripper]
    data.gripper = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/ArmGoal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2132b678d6b320fdf79bc08b99d42769';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
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
    # This message describes the ARM action offered by the PERCHING ARM
    
    uint8 command                                 # What to do
    uint8 ARM_STOP            = 0                 # Stop the arm (vel = 0)
    uint8 ARM_DEPLOY          = 1                 # Deploy the arm
    uint8 ARM_STOW            = 2                 # Retract the arm
    uint8 ARM_PAN             = 3                 # Pan the arm
    uint8 ARM_TILT            = 4                 # Tilt the arm
    uint8 ARM_MOVE            = 5                 # Pan and tilt the
    uint8 GRIPPER_SET         = 6                 # Set the gripper value
    uint8 GRIPPER_OPEN        = 7                 # Open the gripper
    uint8 GRIPPER_CLOSE       = 8                 # Close the gripper
    uint8 DISABLE_SERVO       = 9                 # Disable the servos
    
    float32 pan                                   # PAN from -90 to +90
    float32 tilt                                  # TILT from -120 to +90
    float32 gripper                               # SET from 20 to 45
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ArmGoal(null);
    if (msg.command !== undefined) {
      resolved.command = msg.command;
    }
    else {
      resolved.command = 0
    }

    if (msg.pan !== undefined) {
      resolved.pan = msg.pan;
    }
    else {
      resolved.pan = 0.0
    }

    if (msg.tilt !== undefined) {
      resolved.tilt = msg.tilt;
    }
    else {
      resolved.tilt = 0.0
    }

    if (msg.gripper !== undefined) {
      resolved.gripper = msg.gripper;
    }
    else {
      resolved.gripper = 0.0
    }

    return resolved;
    }
};

// Constants for message
ArmGoal.Constants = {
  ARM_STOP: 0,
  ARM_DEPLOY: 1,
  ARM_STOW: 2,
  ARM_PAN: 3,
  ARM_TILT: 4,
  ARM_MOVE: 5,
  GRIPPER_SET: 6,
  GRIPPER_OPEN: 7,
  GRIPPER_CLOSE: 8,
  DISABLE_SERVO: 9,
}

module.exports = ArmGoal;
