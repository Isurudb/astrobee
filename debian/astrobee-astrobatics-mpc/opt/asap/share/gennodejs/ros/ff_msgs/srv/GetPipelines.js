// Auto-generated. Do not edit!

// (in-package ff_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let LocalizationPipeline = require('../msg/LocalizationPipeline.js');

//-----------------------------------------------------------

class GetPipelinesRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetPipelinesRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPipelinesRequest
    let len;
    let data = new GetPipelinesRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ff_msgs/GetPipelinesRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetPipelinesRequest(null);
    return resolved;
    }
};

class GetPipelinesResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pipelines = null;
    }
    else {
      if (initObj.hasOwnProperty('pipelines')) {
        this.pipelines = initObj.pipelines
      }
      else {
        this.pipelines = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetPipelinesResponse
    // Serialize message field [pipelines]
    // Serialize the length for message field [pipelines]
    bufferOffset = _serializer.uint32(obj.pipelines.length, buffer, bufferOffset);
    obj.pipelines.forEach((val) => {
      bufferOffset = LocalizationPipeline.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPipelinesResponse
    let len;
    let data = new GetPipelinesResponse(null);
    // Deserialize message field [pipelines]
    // Deserialize array length for message field [pipelines]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.pipelines = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.pipelines[i] = LocalizationPipeline.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.pipelines.forEach((val) => {
      length += LocalizationPipeline.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ff_msgs/GetPipelinesResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4fb31d141d0f152e9301905ffcaa8f48';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    ff_msgs/LocalizationPipeline[] pipelines
    
    
    ================================================================================
    MSG: ff_msgs/LocalizationPipeline
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
    # Information about a pipeline
    
    string id                     # Short id for the pipeline
    uint8 mode                    # EKF mode for the pipeline
    string name                   # Long name for the pipe
    bool requires_filter          # Does this pipeline require the EKF
    bool requires_optical_flow    # Does this pipeline require optical flow
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetPipelinesResponse(null);
    if (msg.pipelines !== undefined) {
      resolved.pipelines = new Array(msg.pipelines.length);
      for (let i = 0; i < resolved.pipelines.length; ++i) {
        resolved.pipelines[i] = LocalizationPipeline.Resolve(msg.pipelines[i]);
      }
    }
    else {
      resolved.pipelines = []
    }

    return resolved;
    }
};

module.exports = {
  Request: GetPipelinesRequest,
  Response: GetPipelinesResponse,
  md5sum() { return '4fb31d141d0f152e9301905ffcaa8f48'; },
  datatype() { return 'ff_msgs/GetPipelines'; }
};
