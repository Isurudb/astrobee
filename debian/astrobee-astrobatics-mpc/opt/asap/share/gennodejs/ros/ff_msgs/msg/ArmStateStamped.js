// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ArmJointState = require('./ArmJointState.js');
let ArmGripperState = require('./ArmGripperState.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ArmStateStamped {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.joint_state = null;
      this.gripper_state = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('joint_state')) {
        this.joint_state = initObj.joint_state
      }
      else {
        this.joint_state = new ArmJointState();
      }
      if (initObj.hasOwnProperty('gripper_state')) {
        this.gripper_state = initObj.gripper_state
      }
      else {
        this.gripper_state = new ArmGripperState();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ArmStateStamped
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [joint_state]
    bufferOffset = ArmJointState.serialize(obj.joint_state, buffer, bufferOffset);
    // Serialize message field [gripper_state]
    bufferOffset = ArmGripperState.serialize(obj.gripper_state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArmStateStamped
    let len;
    let data = new ArmStateStamped(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [joint_state]
    data.joint_state = ArmJointState.deserialize(buffer, bufferOffset);
    // Deserialize message field [gripper_state]
    data.gripper_state = ArmGripperState.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 2;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/ArmStateStamped';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3861c96e90f30d3bd53dc5e09edfb937';
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
    # ArmState message
    #
    # *MUST* be kept in sync with rapid::ext::astrobee::ArmState
    
    std_msgs/Header header
    
    ff_msgs/ArmJointState joint_state
    ff_msgs/ArmGripperState  gripper_state
    
    
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
    MSG: ff_msgs/ArmJointState
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
    # Arm Joint State enum.
    #
    # *MUST* be kept in sync with rapid::ext::astrobee::ArmState
    
    uint8 UNKNOWN   = 0
    uint8 STOWED    = 1
    uint8 DEPLOYING = 2
    uint8 STOPPED   = 3
    uint8 MOVING    = 4
    uint8 STOWING   = 5
    
    uint8 state
    
    ================================================================================
    MSG: ff_msgs/ArmGripperState
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
    # Arm Gripper State enum
    #
    # *MUST* be kept in sync with rapid::ext::astrobee::ArmState
    
    uint8 UNKNOWN      = 0
    uint8 UNCALIBRATED = 1
    uint8 CALIBRATING  = 2
    uint8 CLOSED       = 3
    uint8 OPEN         = 4
    
    uint8 state
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ArmStateStamped(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.joint_state !== undefined) {
      resolved.joint_state = ArmJointState.Resolve(msg.joint_state)
    }
    else {
      resolved.joint_state = new ArmJointState()
    }

    if (msg.gripper_state !== undefined) {
      resolved.gripper_state = ArmGripperState.Resolve(msg.gripper_state)
    }
    else {
      resolved.gripper_state = new ArmGripperState()
    }

    return resolved;
    }
};

module.exports = ArmStateStamped;
