// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ControlState = require('./ControlState.js');

//-----------------------------------------------------------

class ControlGoal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.command = null;
      this.segment = null;
    }
    else {
      if (initObj.hasOwnProperty('command')) {
        this.command = initObj.command
      }
      else {
        this.command = 0;
      }
      if (initObj.hasOwnProperty('segment')) {
        this.segment = initObj.segment
      }
      else {
        this.segment = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControlGoal
    // Serialize message field [command]
    bufferOffset = _serializer.uint8(obj.command, buffer, bufferOffset);
    // Serialize message field [segment]
    // Serialize the length for message field [segment]
    bufferOffset = _serializer.uint32(obj.segment.length, buffer, bufferOffset);
    obj.segment.forEach((val) => {
      bufferOffset = ControlState.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControlGoal
    let len;
    let data = new ControlGoal(null);
    // Deserialize message field [command]
    data.command = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [segment]
    // Deserialize array length for message field [segment]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.segment = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.segment[i] = ControlState.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 160 * object.segment.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/ControlGoal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cb33d16997b599aafbc0b0932c171b92';
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
    # This message describes the CONTROL action offered by GNC::WRAPPER
    
    uint8 command                               # STOP, IDLE, NOMINAL
    uint8 STOP    = 0
    uint8 IDLE    = 1
    uint8 NOMINAL = 2
    
    ff_msgs/ControlState[] segment              # NOMINIAL ONLY: Segment
    
    
    ================================================================================
    MSG: ff_msgs/ControlState
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
    # Full state vector containing Time, Pose, Vel, and Accel
    # 
    # when {time}
    # flight_mode {string} - disctates, gains, tolerances, etc.
    # pose {Point position, Quaternion orientation}
    # twist {Vector3 linear, Vector3 angular}
    # accel {Vector3 linear, Vector3 angular}
    
    time when
    geometry_msgs/Pose pose
    geometry_msgs/Twist twist
    geometry_msgs/Twist accel
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: geometry_msgs/Twist
    # This expresses velocity in free space broken into its linear and angular parts.
    Vector3  linear
    Vector3  angular
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControlGoal(null);
    if (msg.command !== undefined) {
      resolved.command = msg.command;
    }
    else {
      resolved.command = 0
    }

    if (msg.segment !== undefined) {
      resolved.segment = new Array(msg.segment.length);
      for (let i = 0; i < resolved.segment.length; ++i) {
        resolved.segment[i] = ControlState.Resolve(msg.segment[i]);
      }
    }
    else {
      resolved.segment = []
    }

    return resolved;
    }
};

// Constants for message
ControlGoal.Constants = {
  STOP: 0,
  IDLE: 1,
  NOMINAL: 2,
}

module.exports = ControlGoal;