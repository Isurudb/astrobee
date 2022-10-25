// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SaveSettings {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.topic_name = null;
      this.downlinkOption = null;
      this.frequency = null;
    }
    else {
      if (initObj.hasOwnProperty('topic_name')) {
        this.topic_name = initObj.topic_name
      }
      else {
        this.topic_name = '';
      }
      if (initObj.hasOwnProperty('downlinkOption')) {
        this.downlinkOption = initObj.downlinkOption
      }
      else {
        this.downlinkOption = 0;
      }
      if (initObj.hasOwnProperty('frequency')) {
        this.frequency = initObj.frequency
      }
      else {
        this.frequency = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SaveSettings
    // Serialize message field [topic_name]
    bufferOffset = _serializer.string(obj.topic_name, buffer, bufferOffset);
    // Serialize message field [downlinkOption]
    bufferOffset = _serializer.uint8(obj.downlinkOption, buffer, bufferOffset);
    // Serialize message field [frequency]
    bufferOffset = _serializer.float32(obj.frequency, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SaveSettings
    let len;
    let data = new SaveSettings(null);
    // Deserialize message field [topic_name]
    data.topic_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [downlinkOption]
    data.downlinkOption = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [frequency]
    data.frequency = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.topic_name.length;
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/SaveSettings';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '87300656673b0987cb5b546a70fa697f';
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
    const resolved = new SaveSettings(null);
    if (msg.topic_name !== undefined) {
      resolved.topic_name = msg.topic_name;
    }
    else {
      resolved.topic_name = ''
    }

    if (msg.downlinkOption !== undefined) {
      resolved.downlinkOption = msg.downlinkOption;
    }
    else {
      resolved.downlinkOption = 0
    }

    if (msg.frequency !== undefined) {
      resolved.frequency = msg.frequency;
    }
    else {
      resolved.frequency = 0.0
    }

    return resolved;
    }
};

// Constants for message
SaveSettings.Constants = {
  IMMEDIATE: 0,
  DELAYED: 1,
}

module.exports = SaveSettings;
