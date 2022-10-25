// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class TimeSyncStamped {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.remote_processor = null;
      this.mlp_time = null;
      this.remote_time = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('remote_processor')) {
        this.remote_processor = initObj.remote_processor
      }
      else {
        this.remote_processor = '';
      }
      if (initObj.hasOwnProperty('mlp_time')) {
        this.mlp_time = initObj.mlp_time
      }
      else {
        this.mlp_time = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('remote_time')) {
        this.remote_time = initObj.remote_time
      }
      else {
        this.remote_time = {secs: 0, nsecs: 0};
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TimeSyncStamped
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [remote_processor]
    bufferOffset = _serializer.string(obj.remote_processor, buffer, bufferOffset);
    // Serialize message field [mlp_time]
    bufferOffset = _serializer.time(obj.mlp_time, buffer, bufferOffset);
    // Serialize message field [remote_time]
    bufferOffset = _serializer.time(obj.remote_time, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TimeSyncStamped
    let len;
    let data = new TimeSyncStamped(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [remote_processor]
    data.remote_processor = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [mlp_time]
    data.mlp_time = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [remote_time]
    data.remote_time = _deserializer.time(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.remote_processor.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/TimeSyncStamped';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '37bae83a741b528e46eb78c2ad2d0190';
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
    # Message to send time difference between the mlp and llp
    
    # Header with timestamp
    std_msgs/Header header
    
    # Processor that the heartbeat came from
    string remote_processor
    
    # Current time on the mlp
    time mlp_time
    
    # Time in the incoming heartbeat
    time remote_time
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TimeSyncStamped(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.remote_processor !== undefined) {
      resolved.remote_processor = msg.remote_processor;
    }
    else {
      resolved.remote_processor = ''
    }

    if (msg.mlp_time !== undefined) {
      resolved.mlp_time = msg.mlp_time;
    }
    else {
      resolved.mlp_time = {secs: 0, nsecs: 0}
    }

    if (msg.remote_time !== undefined) {
      resolved.remote_time = msg.remote_time;
    }
    else {
      resolved.remote_time = {secs: 0, nsecs: 0}
    }

    return resolved;
    }
};

module.exports = TimeSyncStamped;
