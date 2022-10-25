// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ImagePoint = require('./ImagePoint.js');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class DepthCorrespondence {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.source_image_point = null;
      this.target_image_point = null;
      this.source_3d_point = null;
      this.target_3d_point = null;
    }
    else {
      if (initObj.hasOwnProperty('source_image_point')) {
        this.source_image_point = initObj.source_image_point
      }
      else {
        this.source_image_point = new ImagePoint();
      }
      if (initObj.hasOwnProperty('target_image_point')) {
        this.target_image_point = initObj.target_image_point
      }
      else {
        this.target_image_point = new ImagePoint();
      }
      if (initObj.hasOwnProperty('source_3d_point')) {
        this.source_3d_point = initObj.source_3d_point
      }
      else {
        this.source_3d_point = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('target_3d_point')) {
        this.target_3d_point = initObj.target_3d_point
      }
      else {
        this.target_3d_point = new geometry_msgs.msg.Point();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DepthCorrespondence
    // Serialize message field [source_image_point]
    bufferOffset = ImagePoint.serialize(obj.source_image_point, buffer, bufferOffset);
    // Serialize message field [target_image_point]
    bufferOffset = ImagePoint.serialize(obj.target_image_point, buffer, bufferOffset);
    // Serialize message field [source_3d_point]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.source_3d_point, buffer, bufferOffset);
    // Serialize message field [target_3d_point]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.target_3d_point, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DepthCorrespondence
    let len;
    let data = new DepthCorrespondence(null);
    // Deserialize message field [source_image_point]
    data.source_image_point = ImagePoint.deserialize(buffer, bufferOffset);
    // Deserialize message field [target_image_point]
    data.target_image_point = ImagePoint.deserialize(buffer, bufferOffset);
    // Deserialize message field [source_3d_point]
    data.source_3d_point = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [target_3d_point]
    data.target_3d_point = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 64;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/DepthCorrespondence';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '869ec9e0747cdb7e1fd25af7fca82639';
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
    ImagePoint source_image_point
    ImagePoint target_image_point
    geometry_msgs/Point source_3d_point
    geometry_msgs/Point target_3d_point
    
    ================================================================================
    MSG: ff_msgs/ImagePoint
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
    
    float32 x
    float32 y
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DepthCorrespondence(null);
    if (msg.source_image_point !== undefined) {
      resolved.source_image_point = ImagePoint.Resolve(msg.source_image_point)
    }
    else {
      resolved.source_image_point = new ImagePoint()
    }

    if (msg.target_image_point !== undefined) {
      resolved.target_image_point = ImagePoint.Resolve(msg.target_image_point)
    }
    else {
      resolved.target_image_point = new ImagePoint()
    }

    if (msg.source_3d_point !== undefined) {
      resolved.source_3d_point = geometry_msgs.msg.Point.Resolve(msg.source_3d_point)
    }
    else {
      resolved.source_3d_point = new geometry_msgs.msg.Point()
    }

    if (msg.target_3d_point !== undefined) {
      resolved.target_3d_point = geometry_msgs.msg.Point.Resolve(msg.target_3d_point)
    }
    else {
      resolved.target_3d_point = new geometry_msgs.msg.Point()
    }

    return resolved;
    }
};

module.exports = DepthCorrespondence;
