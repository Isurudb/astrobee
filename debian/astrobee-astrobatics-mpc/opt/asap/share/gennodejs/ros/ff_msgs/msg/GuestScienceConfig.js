// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let GuestScienceApk = require('./GuestScienceApk.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class GuestScienceConfig {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.serial = null;
      this.apks = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('serial')) {
        this.serial = initObj.serial
      }
      else {
        this.serial = 0;
      }
      if (initObj.hasOwnProperty('apks')) {
        this.apks = initObj.apks
      }
      else {
        this.apks = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GuestScienceConfig
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [serial]
    bufferOffset = _serializer.int64(obj.serial, buffer, bufferOffset);
    // Serialize message field [apks]
    // Serialize the length for message field [apks]
    bufferOffset = _serializer.uint32(obj.apks.length, buffer, bufferOffset);
    obj.apks.forEach((val) => {
      bufferOffset = GuestScienceApk.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GuestScienceConfig
    let len;
    let data = new GuestScienceConfig(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [serial]
    data.serial = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [apks]
    // Deserialize array length for message field [apks]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.apks = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.apks[i] = GuestScienceApk.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.apks.forEach((val) => {
      length += GuestScienceApk.getMessageSize(val);
    });
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/GuestScienceConfig';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c6f632ed2411de4159494a3e7754794e';
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
    # Message used to store a list of guest science APKs and information relevant to
    # the APK
    
    # Header with timestamp
    std_msgs/Header header
    
    # Used for guest science config and state message synchronization on the ground
    int64 serial
    
    ff_msgs/GuestScienceApk[] apks
    
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
    MSG: ff_msgs/GuestScienceApk
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
    # Message used to contain information about a guest science apk
    
    # Full apk name
    string apk_name
    
    # Short (human readable) name of the apk
    string short_name
    
    # Whether the apk is primary or secondary
    bool primary
    
    # List of commands the apk will accept
    ff_msgs/GuestScienceCommand[] commands
    
    ================================================================================
    MSG: ff_msgs/GuestScienceCommand
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
    # Message used to store guest science commands
    
    # Name of command
    string name
    
    # Syntax of the command
    string command
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GuestScienceConfig(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.serial !== undefined) {
      resolved.serial = msg.serial;
    }
    else {
      resolved.serial = 0
    }

    if (msg.apks !== undefined) {
      resolved.apks = new Array(msg.apks.length);
      for (let i = 0; i < resolved.apks.length; ++i) {
        resolved.apks[i] = GuestScienceApk.Resolve(msg.apks[i]);
      }
    }
    else {
      resolved.apks = []
    }

    return resolved;
    }
};

module.exports = GuestScienceConfig;
