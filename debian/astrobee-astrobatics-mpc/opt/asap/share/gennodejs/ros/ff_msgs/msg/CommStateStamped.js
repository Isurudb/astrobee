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

class CommStateStamped {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.wirelessConnected = null;
      this.apName = null;
      this.bssid = null;
      this.rssi = null;
      this.frequency = null;
      this.channel = null;
      this.lanConnected = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('wirelessConnected')) {
        this.wirelessConnected = initObj.wirelessConnected
      }
      else {
        this.wirelessConnected = false;
      }
      if (initObj.hasOwnProperty('apName')) {
        this.apName = initObj.apName
      }
      else {
        this.apName = '';
      }
      if (initObj.hasOwnProperty('bssid')) {
        this.bssid = initObj.bssid
      }
      else {
        this.bssid = '';
      }
      if (initObj.hasOwnProperty('rssi')) {
        this.rssi = initObj.rssi
      }
      else {
        this.rssi = 0.0;
      }
      if (initObj.hasOwnProperty('frequency')) {
        this.frequency = initObj.frequency
      }
      else {
        this.frequency = 0.0;
      }
      if (initObj.hasOwnProperty('channel')) {
        this.channel = initObj.channel
      }
      else {
        this.channel = 0;
      }
      if (initObj.hasOwnProperty('lanConnected')) {
        this.lanConnected = initObj.lanConnected
      }
      else {
        this.lanConnected = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CommStateStamped
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [wirelessConnected]
    bufferOffset = _serializer.bool(obj.wirelessConnected, buffer, bufferOffset);
    // Serialize message field [apName]
    bufferOffset = _serializer.string(obj.apName, buffer, bufferOffset);
    // Serialize message field [bssid]
    bufferOffset = _serializer.string(obj.bssid, buffer, bufferOffset);
    // Serialize message field [rssi]
    bufferOffset = _serializer.float32(obj.rssi, buffer, bufferOffset);
    // Serialize message field [frequency]
    bufferOffset = _serializer.float32(obj.frequency, buffer, bufferOffset);
    // Serialize message field [channel]
    bufferOffset = _serializer.uint16(obj.channel, buffer, bufferOffset);
    // Serialize message field [lanConnected]
    bufferOffset = _serializer.bool(obj.lanConnected, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CommStateStamped
    let len;
    let data = new CommStateStamped(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [wirelessConnected]
    data.wirelessConnected = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [apName]
    data.apName = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [bssid]
    data.bssid = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [rssi]
    data.rssi = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [frequency]
    data.frequency = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [channel]
    data.channel = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [lanConnected]
    data.lanConnected = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.apName.length;
    length += object.bssid.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/CommStateStamped';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bfb1cf1d26d8d3be4813192cd797e19f';
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
    # CommState, based off of rapid::ext::astrobee::CommState
    
    std_msgs/Header header
    
    bool wirelessConnected
    string apName
    string bssid
    float32 rssi
    float32 frequency
    uint16 channel
    
    bool lanConnected
    
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
    const resolved = new CommStateStamped(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.wirelessConnected !== undefined) {
      resolved.wirelessConnected = msg.wirelessConnected;
    }
    else {
      resolved.wirelessConnected = false
    }

    if (msg.apName !== undefined) {
      resolved.apName = msg.apName;
    }
    else {
      resolved.apName = ''
    }

    if (msg.bssid !== undefined) {
      resolved.bssid = msg.bssid;
    }
    else {
      resolved.bssid = ''
    }

    if (msg.rssi !== undefined) {
      resolved.rssi = msg.rssi;
    }
    else {
      resolved.rssi = 0.0
    }

    if (msg.frequency !== undefined) {
      resolved.frequency = msg.frequency;
    }
    else {
      resolved.frequency = 0.0
    }

    if (msg.channel !== undefined) {
      resolved.channel = msg.channel;
    }
    else {
      resolved.channel = 0
    }

    if (msg.lanConnected !== undefined) {
      resolved.lanConnected = msg.lanConnected;
    }
    else {
      resolved.lanConnected = false
    }

    return resolved;
    }
};

module.exports = CommStateStamped;
