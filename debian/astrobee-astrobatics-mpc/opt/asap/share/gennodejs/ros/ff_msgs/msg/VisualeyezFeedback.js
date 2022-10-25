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

class VisualeyezFeedback {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.tcmid = null;
      this.ledid = null;
      this.count = null;
    }
    else {
      if (initObj.hasOwnProperty('tcmid')) {
        this.tcmid = initObj.tcmid
      }
      else {
        this.tcmid = 0;
      }
      if (initObj.hasOwnProperty('ledid')) {
        this.ledid = initObj.ledid
      }
      else {
        this.ledid = 0;
      }
      if (initObj.hasOwnProperty('count')) {
        this.count = initObj.count
      }
      else {
        this.count = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VisualeyezFeedback
    // Serialize message field [tcmid]
    bufferOffset = _serializer.uint8(obj.tcmid, buffer, bufferOffset);
    // Serialize message field [ledid]
    bufferOffset = _serializer.uint8(obj.ledid, buffer, bufferOffset);
    // Serialize message field [count]
    bufferOffset = _serializer.uint32(obj.count, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VisualeyezFeedback
    let len;
    let data = new VisualeyezFeedback(null);
    // Deserialize message field [tcmid]
    data.tcmid = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [ledid]
    data.ledid = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [count]
    data.count = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/VisualeyezFeedback';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2ce06a8bb76a1a67ecd18953ba9bbf84';
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
    # Visualeyez feedback.
    
    uint8 tcmid                         # Transmission control module ID
    uint8 ledid                         # Light emitting diode ID
    uint32 count                        # Number of valid measurements
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VisualeyezFeedback(null);
    if (msg.tcmid !== undefined) {
      resolved.tcmid = msg.tcmid;
    }
    else {
      resolved.tcmid = 0
    }

    if (msg.ledid !== undefined) {
      resolved.ledid = msg.ledid;
    }
    else {
      resolved.ledid = 0
    }

    if (msg.count !== undefined) {
      resolved.count = msg.count;
    }
    else {
      resolved.count = 0
    }

    return resolved;
    }
};

module.exports = VisualeyezFeedback;
