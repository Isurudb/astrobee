// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Feature2d = require('./Feature2d.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Feature2dArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.camera_id = null;
      this.feature_array = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('camera_id')) {
        this.camera_id = initObj.camera_id
      }
      else {
        this.camera_id = 0;
      }
      if (initObj.hasOwnProperty('feature_array')) {
        this.feature_array = initObj.feature_array
      }
      else {
        this.feature_array = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Feature2dArray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [camera_id]
    bufferOffset = _serializer.uint32(obj.camera_id, buffer, bufferOffset);
    // Serialize message field [feature_array]
    // Serialize the length for message field [feature_array]
    bufferOffset = _serializer.uint32(obj.feature_array.length, buffer, bufferOffset);
    obj.feature_array.forEach((val) => {
      bufferOffset = Feature2d.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Feature2dArray
    let len;
    let data = new Feature2dArray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [camera_id]
    data.camera_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [feature_array]
    // Deserialize array length for message field [feature_array]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.feature_array = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.feature_array[i] = Feature2d.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 10 * object.feature_array.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/Feature2dArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '579cb05879a7a1292a35750f014c3208';
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
    # An observation of image points and associated ids, for optical flow.
    
    Header header # header with timestamp
    uint32 camera_id # image ID, linked to registration pulse
    Feature2d[] feature_array # list of observed features
    
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
    MSG: ff_msgs/Feature2d
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
    # A single observation of a feature, with an ID and coordinates.
    # Used for an optical flow feature.
    
    uint16 id # feature ID
    float32 x # feature x coordinate
    float32 y # feature y coordinate
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Feature2dArray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.camera_id !== undefined) {
      resolved.camera_id = msg.camera_id;
    }
    else {
      resolved.camera_id = 0
    }

    if (msg.feature_array !== undefined) {
      resolved.feature_array = new Array(msg.feature_array.length);
      for (let i = 0; i < resolved.feature_array.length; ++i) {
        resolved.feature_array[i] = Feature2d.Resolve(msg.feature_array[i]);
      }
    }
    else {
      resolved.feature_array = []
    }

    return resolved;
    }
};

module.exports = Feature2dArray;
