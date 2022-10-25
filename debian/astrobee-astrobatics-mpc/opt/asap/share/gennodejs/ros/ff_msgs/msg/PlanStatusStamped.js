// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let AckStatus = require('./AckStatus.js');
let Status = require('./Status.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class PlanStatusStamped {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.name = null;
      this.point = null;
      this.command = null;
      this.status = null;
      this.history = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('point')) {
        this.point = initObj.point
      }
      else {
        this.point = 0;
      }
      if (initObj.hasOwnProperty('command')) {
        this.command = initObj.command
      }
      else {
        this.command = 0;
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = new AckStatus();
      }
      if (initObj.hasOwnProperty('history')) {
        this.history = initObj.history
      }
      else {
        this.history = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlanStatusStamped
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [point]
    bufferOffset = _serializer.int32(obj.point, buffer, bufferOffset);
    // Serialize message field [command]
    bufferOffset = _serializer.int32(obj.command, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = AckStatus.serialize(obj.status, buffer, bufferOffset);
    // Serialize message field [history]
    // Serialize the length for message field [history]
    bufferOffset = _serializer.uint32(obj.history.length, buffer, bufferOffset);
    obj.history.forEach((val) => {
      bufferOffset = Status.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlanStatusStamped
    let len;
    let data = new PlanStatusStamped(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [point]
    data.point = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [command]
    data.command = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = AckStatus.deserialize(buffer, bufferOffset);
    // Deserialize message field [history]
    // Deserialize array length for message field [history]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.history = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.history[i] = Status.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.name.length;
    length += 13 * object.history.length;
    return length + 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/PlanStatusStamped';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6b462b22b23dabb22643884c3aecd09b';
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
    # Plan status message. Based off of PlanStatus from DDS. Note that while in
    # ROS we use an unbounded array, we are actually limited to ~64 previous
    # status messages in the DDS type.
    
    # Header with timestamp
    std_msgs/Header header
    
    # Name of plan
    string name
    
    int32 point                 # Current station or segment
    int32 command               # Current subcommand within station/segment or -1
    ff_msgs/AckStatus status    # Status of the currently executing plan element
    
    ff_msgs/Status[] history    # Completion status of the last 64 plan elements
    
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
    MSG: ff_msgs/AckStatus
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
    # Command status. Based off AckStatus in RAPID DDS
    
    uint8 QUEUED = 0      # Command is in a queue and waiting to be executed
    uint8 EXECUTING = 1   # Command is being executed
    uint8 REQUEUED = 2    # Command is paused and waiting to be restarted 
    uint8 COMPLETED = 3   # Command is finished
    
    uint8 status          # Command status
    
    ================================================================================
    MSG: ff_msgs/Status
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
    # Sub-type for a command's status in a PlanStatus' history.
    
    int32 point                         # Station or segment
    int32 command                       # Subcommand within station/segment or -1
    int32 duration                      # How long it took
    ff_msgs/AckCompletedStatus status   # The completion status
    
    ================================================================================
    MSG: ff_msgs/AckCompletedStatus
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
    # Completed command status. Based on AckCompletedStatus from RAPID DDS
    
    uint8 NOT = 0           # Command not completed
    uint8 OK = 1            # Command completed successfully
    uint8 BAD_SYNTAX = 2    # Command not recognized, bad parameters, etc.
    uint8 EXEC_FAILED = 3   # Command failed to execute
    uint8 CANCELED = 4      # Command was canceled by operator
    
    # Completed command status
    uint8 status
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PlanStatusStamped(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.point !== undefined) {
      resolved.point = msg.point;
    }
    else {
      resolved.point = 0
    }

    if (msg.command !== undefined) {
      resolved.command = msg.command;
    }
    else {
      resolved.command = 0
    }

    if (msg.status !== undefined) {
      resolved.status = AckStatus.Resolve(msg.status)
    }
    else {
      resolved.status = new AckStatus()
    }

    if (msg.history !== undefined) {
      resolved.history = new Array(msg.history.length);
      for (let i = 0; i < resolved.history.length; ++i) {
        resolved.history[i] = Status.Resolve(msg.history[i]);
      }
    }
    else {
      resolved.history = []
    }

    return resolved;
    }
};

module.exports = PlanStatusStamped;
