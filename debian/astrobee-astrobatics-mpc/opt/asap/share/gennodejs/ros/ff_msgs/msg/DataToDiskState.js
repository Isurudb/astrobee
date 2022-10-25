// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SaveSettings = require('./SaveSettings.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class DataToDiskState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.name = null;
      this.recording = null;
      this.topic_save_settings = null;
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
      if (initObj.hasOwnProperty('recording')) {
        this.recording = initObj.recording
      }
      else {
        this.recording = false;
      }
      if (initObj.hasOwnProperty('topic_save_settings')) {
        this.topic_save_settings = initObj.topic_save_settings
      }
      else {
        this.topic_save_settings = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DataToDiskState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [recording]
    bufferOffset = _serializer.bool(obj.recording, buffer, bufferOffset);
    // Serialize message field [topic_save_settings]
    // Serialize the length for message field [topic_save_settings]
    bufferOffset = _serializer.uint32(obj.topic_save_settings.length, buffer, bufferOffset);
    obj.topic_save_settings.forEach((val) => {
      bufferOffset = SaveSettings.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DataToDiskState
    let len;
    let data = new DataToDiskState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [recording]
    data.recording = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [topic_save_settings]
    // Deserialize array length for message field [topic_save_settings]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.topic_save_settings = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.topic_save_settings[i] = SaveSettings.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.name.length;
    object.topic_save_settings.forEach((val) => {
      length += SaveSettings.getMessageSize(val);
    });
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/DataToDiskState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '68d7ec16d4c7bc2b6e1a00776a76b4f7';
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
    
    # Data to disk state message used to let ground operators know which topics
    # are currently being recorded.
    
    # Header with timestamp
    std_msgs/Header header
    
    # Name of the latest data to disk file uploaded from the ground
    string name
    
    # Whether the data bagger is recording a bag or not
    bool recording
    
    # An array containing information about the topics being recorded
    ff_msgs/SaveSettings[] topic_save_settings
    
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
    MSG: ff_msgs/SaveSettings
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
    
    # The save settings message contains information about the topics currently
    # being recorded.
    
    # Name of topic
    string topic_name
    
    # Topic saved to disk; upon docking it is downlinked
    uint8 IMMEDIATE   = 0
    
    # Topic saved to disk; upon docking it is transferred to ISS server for later
    # downlink
    uint8 DELAYED     = 1
    
    # Downlink option indicates if and when the data in the rostopic is downlinked
    uint8 downlinkOption
    
    # Times per second to save the data (Hz)
    float32 frequency
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DataToDiskState(null);
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

    if (msg.recording !== undefined) {
      resolved.recording = msg.recording;
    }
    else {
      resolved.recording = false
    }

    if (msg.topic_save_settings !== undefined) {
      resolved.topic_save_settings = new Array(msg.topic_save_settings.length);
      for (let i = 0; i < resolved.topic_save_settings.length; ++i) {
        resolved.topic_save_settings[i] = SaveSettings.Resolve(msg.topic_save_settings[i]);
      }
    }
    else {
      resolved.topic_save_settings = []
    }

    return resolved;
    }
};

module.exports = DataToDiskState;
