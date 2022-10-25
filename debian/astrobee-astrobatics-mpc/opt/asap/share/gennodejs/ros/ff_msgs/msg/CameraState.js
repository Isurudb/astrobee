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

class CameraState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.camera_name = null;
      this.streaming = null;
      this.stream_width = null;
      this.stream_height = null;
      this.stream_rate = null;
      this.recording = null;
      this.record_width = null;
      this.record_height = null;
      this.record_rate = null;
      this.bandwidth = null;
    }
    else {
      if (initObj.hasOwnProperty('camera_name')) {
        this.camera_name = initObj.camera_name
      }
      else {
        this.camera_name = '';
      }
      if (initObj.hasOwnProperty('streaming')) {
        this.streaming = initObj.streaming
      }
      else {
        this.streaming = false;
      }
      if (initObj.hasOwnProperty('stream_width')) {
        this.stream_width = initObj.stream_width
      }
      else {
        this.stream_width = 0;
      }
      if (initObj.hasOwnProperty('stream_height')) {
        this.stream_height = initObj.stream_height
      }
      else {
        this.stream_height = 0;
      }
      if (initObj.hasOwnProperty('stream_rate')) {
        this.stream_rate = initObj.stream_rate
      }
      else {
        this.stream_rate = 0.0;
      }
      if (initObj.hasOwnProperty('recording')) {
        this.recording = initObj.recording
      }
      else {
        this.recording = false;
      }
      if (initObj.hasOwnProperty('record_width')) {
        this.record_width = initObj.record_width
      }
      else {
        this.record_width = 0;
      }
      if (initObj.hasOwnProperty('record_height')) {
        this.record_height = initObj.record_height
      }
      else {
        this.record_height = 0;
      }
      if (initObj.hasOwnProperty('record_rate')) {
        this.record_rate = initObj.record_rate
      }
      else {
        this.record_rate = 0.0;
      }
      if (initObj.hasOwnProperty('bandwidth')) {
        this.bandwidth = initObj.bandwidth
      }
      else {
        this.bandwidth = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CameraState
    // Serialize message field [camera_name]
    bufferOffset = _serializer.string(obj.camera_name, buffer, bufferOffset);
    // Serialize message field [streaming]
    bufferOffset = _serializer.bool(obj.streaming, buffer, bufferOffset);
    // Serialize message field [stream_width]
    bufferOffset = _serializer.uint16(obj.stream_width, buffer, bufferOffset);
    // Serialize message field [stream_height]
    bufferOffset = _serializer.uint16(obj.stream_height, buffer, bufferOffset);
    // Serialize message field [stream_rate]
    bufferOffset = _serializer.float32(obj.stream_rate, buffer, bufferOffset);
    // Serialize message field [recording]
    bufferOffset = _serializer.bool(obj.recording, buffer, bufferOffset);
    // Serialize message field [record_width]
    bufferOffset = _serializer.uint16(obj.record_width, buffer, bufferOffset);
    // Serialize message field [record_height]
    bufferOffset = _serializer.uint16(obj.record_height, buffer, bufferOffset);
    // Serialize message field [record_rate]
    bufferOffset = _serializer.float32(obj.record_rate, buffer, bufferOffset);
    // Serialize message field [bandwidth]
    bufferOffset = _serializer.float32(obj.bandwidth, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CameraState
    let len;
    let data = new CameraState(null);
    // Deserialize message field [camera_name]
    data.camera_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [streaming]
    data.streaming = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [stream_width]
    data.stream_width = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [stream_height]
    data.stream_height = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [stream_rate]
    data.stream_rate = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [recording]
    data.recording = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [record_width]
    data.record_width = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [record_height]
    data.record_height = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [record_rate]
    data.record_rate = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bandwidth]
    data.bandwidth = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.camera_name.length;
    return length + 26;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/CameraState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '644cfd14384d17cf28911b625a446f53';
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
    # CameraState message, *MUST* be kept in sync with camera portion of
    # rapid::ext::astrobee::TelemetryState
    
    # nav_cam, dock_cam, etc.
    string camera_name
    
    # streaming to ground
    bool streaming
    
    # image width
    uint16 stream_width
    # image height
    uint16 stream_height
    # Rate in Hz
    float32 stream_rate
    
    # recording to disk
    bool recording
    
    # image width
    uint16 record_width
    # image height
    uint16 record_height
    # Rate in Hz
    float32 record_rate
    
    # only for sci cam
    float32 bandwidth
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CameraState(null);
    if (msg.camera_name !== undefined) {
      resolved.camera_name = msg.camera_name;
    }
    else {
      resolved.camera_name = ''
    }

    if (msg.streaming !== undefined) {
      resolved.streaming = msg.streaming;
    }
    else {
      resolved.streaming = false
    }

    if (msg.stream_width !== undefined) {
      resolved.stream_width = msg.stream_width;
    }
    else {
      resolved.stream_width = 0
    }

    if (msg.stream_height !== undefined) {
      resolved.stream_height = msg.stream_height;
    }
    else {
      resolved.stream_height = 0
    }

    if (msg.stream_rate !== undefined) {
      resolved.stream_rate = msg.stream_rate;
    }
    else {
      resolved.stream_rate = 0.0
    }

    if (msg.recording !== undefined) {
      resolved.recording = msg.recording;
    }
    else {
      resolved.recording = false
    }

    if (msg.record_width !== undefined) {
      resolved.record_width = msg.record_width;
    }
    else {
      resolved.record_width = 0
    }

    if (msg.record_height !== undefined) {
      resolved.record_height = msg.record_height;
    }
    else {
      resolved.record_height = 0
    }

    if (msg.record_rate !== undefined) {
      resolved.record_rate = msg.record_rate;
    }
    else {
      resolved.record_rate = 0.0
    }

    if (msg.bandwidth !== undefined) {
      resolved.bandwidth = msg.bandwidth;
    }
    else {
      resolved.bandwidth = 0.0
    }

    return resolved;
    }
};

module.exports = CameraState;
