// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Odometry = require('./Odometry.js');
let DepthCorrespondence = require('./DepthCorrespondence.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class DepthOdometry {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.odometry = null;
      this.correspondences = null;
      this.valid_image_points = null;
      this.valid_points_3d = null;
      this.runtime = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('odometry')) {
        this.odometry = initObj.odometry
      }
      else {
        this.odometry = new Odometry();
      }
      if (initObj.hasOwnProperty('correspondences')) {
        this.correspondences = initObj.correspondences
      }
      else {
        this.correspondences = [];
      }
      if (initObj.hasOwnProperty('valid_image_points')) {
        this.valid_image_points = initObj.valid_image_points
      }
      else {
        this.valid_image_points = false;
      }
      if (initObj.hasOwnProperty('valid_points_3d')) {
        this.valid_points_3d = initObj.valid_points_3d
      }
      else {
        this.valid_points_3d = false;
      }
      if (initObj.hasOwnProperty('runtime')) {
        this.runtime = initObj.runtime
      }
      else {
        this.runtime = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DepthOdometry
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [odometry]
    bufferOffset = Odometry.serialize(obj.odometry, buffer, bufferOffset);
    // Serialize message field [correspondences]
    // Serialize the length for message field [correspondences]
    bufferOffset = _serializer.uint32(obj.correspondences.length, buffer, bufferOffset);
    obj.correspondences.forEach((val) => {
      bufferOffset = DepthCorrespondence.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [valid_image_points]
    bufferOffset = _serializer.bool(obj.valid_image_points, buffer, bufferOffset);
    // Serialize message field [valid_points_3d]
    bufferOffset = _serializer.bool(obj.valid_points_3d, buffer, bufferOffset);
    // Serialize message field [runtime]
    bufferOffset = _serializer.float32(obj.runtime, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DepthOdometry
    let len;
    let data = new DepthOdometry(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [odometry]
    data.odometry = Odometry.deserialize(buffer, bufferOffset);
    // Deserialize message field [correspondences]
    // Deserialize array length for message field [correspondences]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.correspondences = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.correspondences[i] = DepthCorrespondence.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [valid_image_points]
    data.valid_image_points = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [valid_points_3d]
    data.valid_points_3d = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [runtime]
    data.runtime = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 64 * object.correspondences.length;
    return length + 714;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/DepthOdometry';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd00049c091a5ccf31e3eef01d010e9fa';
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
    
    Header header
    ff_msgs/Odometry odometry
    ff_msgs/DepthCorrespondence[] correspondences
    bool valid_image_points
    bool valid_points_3d
    float32 runtime
    
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
    MSG: ff_msgs/Odometry
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
    
    time source_time 
    time target_time
    geometry_msgs/PoseWithCovariance sensor_F_source_T_target
    geometry_msgs/PoseWithCovariance body_F_source_T_target
    
    ================================================================================
    MSG: geometry_msgs/PoseWithCovariance
    # This represents a pose in free space with uncertainty.
    
    Pose pose
    
    # Row-major representation of the 6x6 covariance matrix
    # The orientation parameters use a fixed-axis representation.
    # In order, the parameters are:
    # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    float64[36] covariance
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: ff_msgs/DepthCorrespondence
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DepthOdometry(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.odometry !== undefined) {
      resolved.odometry = Odometry.Resolve(msg.odometry)
    }
    else {
      resolved.odometry = new Odometry()
    }

    if (msg.correspondences !== undefined) {
      resolved.correspondences = new Array(msg.correspondences.length);
      for (let i = 0; i < resolved.correspondences.length; ++i) {
        resolved.correspondences[i] = DepthCorrespondence.Resolve(msg.correspondences[i]);
      }
    }
    else {
      resolved.correspondences = []
    }

    if (msg.valid_image_points !== undefined) {
      resolved.valid_image_points = msg.valid_image_points;
    }
    else {
      resolved.valid_image_points = false
    }

    if (msg.valid_points_3d !== undefined) {
      resolved.valid_points_3d = msg.valid_points_3d;
    }
    else {
      resolved.valid_points_3d = false
    }

    if (msg.runtime !== undefined) {
      resolved.runtime = msg.runtime;
    }
    else {
      resolved.runtime = 0.0
    }

    return resolved;
    }
};

module.exports = DepthOdometry;
