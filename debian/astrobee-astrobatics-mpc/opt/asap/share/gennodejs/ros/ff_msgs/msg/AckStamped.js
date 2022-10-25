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
let AckCompletedStatus = require('./AckCompletedStatus.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class AckStamped {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.cmd_id = null;
      this.status = null;
      this.completed_status = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('cmd_id')) {
        this.cmd_id = initObj.cmd_id
      }
      else {
        this.cmd_id = '';
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = new AckStatus();
      }
      if (initObj.hasOwnProperty('completed_status')) {
        this.completed_status = initObj.completed_status
      }
      else {
        this.completed_status = new AckCompletedStatus();
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AckStamped
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [cmd_id]
    bufferOffset = _serializer.string(obj.cmd_id, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = AckStatus.serialize(obj.status, buffer, bufferOffset);
    // Serialize message field [completed_status]
    bufferOffset = AckCompletedStatus.serialize(obj.completed_status, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AckStamped
    let len;
    let data = new AckStamped(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [cmd_id]
    data.cmd_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = AckStatus.deserialize(buffer, bufferOffset);
    // Deserialize message field [completed_status]
    data.completed_status = AckCompletedStatus.deserialize(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.cmd_id.length;
    length += object.message.length;
    return length + 10;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/AckStamped';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cebddec69fd770df54633be444a9187d';
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
    # Message used to send an acknowledgement for commands received. Based off of
    # Ack in RAPID DDS
    
    # Header with timestamp
    std_msgs/Header header
    
    # Id of the command being acknowledged
    string cmd_id
    
    # Status of the command
    ff_msgs/AckStatus status
    
    # Completed status of the command
    ff_msgs/AckCompletedStatus completed_status
    
    # If the command fails to execute, message will contain information on why it
    # failed.
    string message
    
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
    const resolved = new AckStamped(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.cmd_id !== undefined) {
      resolved.cmd_id = msg.cmd_id;
    }
    else {
      resolved.cmd_id = ''
    }

    if (msg.status !== undefined) {
      resolved.status = AckStatus.Resolve(msg.status)
    }
    else {
      resolved.status = new AckStatus()
    }

    if (msg.completed_status !== undefined) {
      resolved.completed_status = AckCompletedStatus.Resolve(msg.completed_status)
    }
    else {
      resolved.completed_status = new AckCompletedStatus()
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    return resolved;
    }
};

module.exports = AckStamped;
