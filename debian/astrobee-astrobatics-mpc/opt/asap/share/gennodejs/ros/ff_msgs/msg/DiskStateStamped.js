// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let DiskState = require('./DiskState.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class DiskStateStamped {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.processor_name = null;
      this.disks = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('processor_name')) {
        this.processor_name = initObj.processor_name
      }
      else {
        this.processor_name = '';
      }
      if (initObj.hasOwnProperty('disks')) {
        this.disks = initObj.disks
      }
      else {
        this.disks = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DiskStateStamped
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [processor_name]
    bufferOffset = _serializer.string(obj.processor_name, buffer, bufferOffset);
    // Serialize message field [disks]
    // Serialize the length for message field [disks]
    bufferOffset = _serializer.uint32(obj.disks.length, buffer, bufferOffset);
    obj.disks.forEach((val) => {
      bufferOffset = DiskState.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DiskStateStamped
    let len;
    let data = new DiskStateStamped(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [processor_name]
    data.processor_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [disks]
    // Deserialize array length for message field [disks]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.disks = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.disks[i] = DiskState.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.processor_name.length;
    object.disks.forEach((val) => {
      length += DiskState.getMessageSize(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/DiskStateStamped';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'af262ed457fa1b453f85c47b9f5c607b';
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
    # This message describes the state of a filesystem within astrobee
    # Based off of DiskState from rapid::ext::astrobee
    
    # Header with timestamp
    std_msgs/Header header
    
    string processor_name       # Processor name, either llp, mlp, or hlp
    
    # Information on the mounted filesystem on the processor
    ff_msgs/DiskState[] disks
    
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
    MSG: ff_msgs/DiskState
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
    # This message describes the state of a filesystem within astrobee
    # Based off of DiskState from rapid::ext::astrobee
    
    string path       # The pathname of the file within the mounted filesystem
    uint64 capacity   # The size of the filesystem
    uint64 used       # The amount of the filesystem being used
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DiskStateStamped(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.processor_name !== undefined) {
      resolved.processor_name = msg.processor_name;
    }
    else {
      resolved.processor_name = ''
    }

    if (msg.disks !== undefined) {
      resolved.disks = new Array(msg.disks.length);
      for (let i = 0; i < resolved.disks.length; ++i) {
        resolved.disks[i] = DiskState.Resolve(msg.disks[i]);
      }
    }
    else {
      resolved.disks = []
    }

    return resolved;
    }
};

module.exports = DiskStateStamped;
