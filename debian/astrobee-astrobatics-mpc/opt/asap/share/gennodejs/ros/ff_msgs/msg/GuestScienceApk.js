// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let GuestScienceCommand = require('./GuestScienceCommand.js');

//-----------------------------------------------------------

class GuestScienceApk {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.apk_name = null;
      this.short_name = null;
      this.primary = null;
      this.commands = null;
    }
    else {
      if (initObj.hasOwnProperty('apk_name')) {
        this.apk_name = initObj.apk_name
      }
      else {
        this.apk_name = '';
      }
      if (initObj.hasOwnProperty('short_name')) {
        this.short_name = initObj.short_name
      }
      else {
        this.short_name = '';
      }
      if (initObj.hasOwnProperty('primary')) {
        this.primary = initObj.primary
      }
      else {
        this.primary = false;
      }
      if (initObj.hasOwnProperty('commands')) {
        this.commands = initObj.commands
      }
      else {
        this.commands = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GuestScienceApk
    // Serialize message field [apk_name]
    bufferOffset = _serializer.string(obj.apk_name, buffer, bufferOffset);
    // Serialize message field [short_name]
    bufferOffset = _serializer.string(obj.short_name, buffer, bufferOffset);
    // Serialize message field [primary]
    bufferOffset = _serializer.bool(obj.primary, buffer, bufferOffset);
    // Serialize message field [commands]
    // Serialize the length for message field [commands]
    bufferOffset = _serializer.uint32(obj.commands.length, buffer, bufferOffset);
    obj.commands.forEach((val) => {
      bufferOffset = GuestScienceCommand.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GuestScienceApk
    let len;
    let data = new GuestScienceApk(null);
    // Deserialize message field [apk_name]
    data.apk_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [short_name]
    data.short_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [primary]
    data.primary = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [commands]
    // Deserialize array length for message field [commands]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.commands = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.commands[i] = GuestScienceCommand.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.apk_name.length;
    length += object.short_name.length;
    object.commands.forEach((val) => {
      length += GuestScienceCommand.getMessageSize(val);
    });
    return length + 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/GuestScienceApk';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8ed1d23e09733f18dbf96d2f9cd798e5';
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
    const resolved = new GuestScienceApk(null);
    if (msg.apk_name !== undefined) {
      resolved.apk_name = msg.apk_name;
    }
    else {
      resolved.apk_name = ''
    }

    if (msg.short_name !== undefined) {
      resolved.short_name = msg.short_name;
    }
    else {
      resolved.short_name = ''
    }

    if (msg.primary !== undefined) {
      resolved.primary = msg.primary;
    }
    else {
      resolved.primary = false
    }

    if (msg.commands !== undefined) {
      resolved.commands = new Array(msg.commands.length);
      for (let i = 0; i < resolved.commands.length; ++i) {
        resolved.commands[i] = GuestScienceCommand.Resolve(msg.commands[i]);
      }
    }
    else {
      resolved.commands = []
    }

    return resolved;
    }
};

module.exports = GuestScienceApk;
