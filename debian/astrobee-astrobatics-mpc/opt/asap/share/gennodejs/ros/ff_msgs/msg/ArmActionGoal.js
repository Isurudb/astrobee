// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ArmGoal = require('./ArmGoal.js');
let actionlib_msgs = _finder('actionlib_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ArmActionGoal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.goal_id = null;
      this.goal = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('goal_id')) {
        this.goal_id = initObj.goal_id
      }
      else {
        this.goal_id = new actionlib_msgs.msg.GoalID();
      }
      if (initObj.hasOwnProperty('goal')) {
        this.goal = initObj.goal
      }
      else {
        this.goal = new ArmGoal();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ArmActionGoal
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [goal_id]
    bufferOffset = actionlib_msgs.msg.GoalID.serialize(obj.goal_id, buffer, bufferOffset);
    // Serialize message field [goal]
    bufferOffset = ArmGoal.serialize(obj.goal, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArmActionGoal
    let len;
    let data = new ArmActionGoal(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [goal_id]
    data.goal_id = actionlib_msgs.msg.GoalID.deserialize(buffer, bufferOffset);
    // Deserialize message field [goal]
    data.goal = ArmGoal.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += actionlib_msgs.msg.GoalID.getMessageSize(object.goal_id);
    return length + 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/ArmActionGoal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '760dabe97c6cc65b883b66a0ca69047f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    
    Header header
    actionlib_msgs/GoalID goal_id
    ArmGoal goal
    
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
    MSG: actionlib_msgs/GoalID
    # The stamp should store the time at which this goal was requested.
    # It is used by an action server when it tries to preempt all
    # goals that were requested before a certain time
    time stamp
    
    # The id provides a way to associate feedback and
    # result message with specific goal requests. The id
    # specified must be unique.
    string id
    
    
    ================================================================================
    MSG: ff_msgs/ArmGoal
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
    const resolved = new ArmActionGoal(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.goal_id !== undefined) {
      resolved.goal_id = actionlib_msgs.msg.GoalID.Resolve(msg.goal_id)
    }
    else {
      resolved.goal_id = new actionlib_msgs.msg.GoalID()
    }

    if (msg.goal !== undefined) {
      resolved.goal = ArmGoal.Resolve(msg.goal)
    }
    else {
      resolved.goal = new ArmGoal()
    }

    return resolved;
    }
};

module.exports = ArmActionGoal;