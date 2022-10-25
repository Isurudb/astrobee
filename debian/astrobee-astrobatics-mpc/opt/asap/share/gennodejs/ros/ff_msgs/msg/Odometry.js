// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Odometry {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.source_time = null;
      this.target_time = null;
      this.sensor_F_source_T_target = null;
      this.body_F_source_T_target = null;
    }
    else {
      if (initObj.hasOwnProperty('source_time')) {
        this.source_time = initObj.source_time
      }
      else {
        this.source_time = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('target_time')) {
        this.target_time = initObj.target_time
      }
      else {
        this.target_time = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('sensor_F_source_T_target')) {
        this.sensor_F_source_T_target = initObj.sensor_F_source_T_target
      }
      else {
        this.sensor_F_source_T_target = new geometry_msgs.msg.PoseWithCovariance();
      }
      if (initObj.hasOwnProperty('body_F_source_T_target')) {
        this.body_F_source_T_target = initObj.body_F_source_T_target
      }
      else {
        this.body_F_source_T_target = new geometry_msgs.msg.PoseWithCovariance();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Odometry
    // Serialize message field [source_time]
    bufferOffset = _serializer.time(obj.source_time, buffer, bufferOffset);
    // Serialize message field [target_time]
    bufferOffset = _serializer.time(obj.target_time, buffer, bufferOffset);
    // Serialize message field [sensor_F_source_T_target]
    bufferOffset = geometry_msgs.msg.PoseWithCovariance.serialize(obj.sensor_F_source_T_target, buffer, bufferOffset);
    // Serialize message field [body_F_source_T_target]
    bufferOffset = geometry_msgs.msg.PoseWithCovariance.serialize(obj.body_F_source_T_target, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Odometry
    let len;
    let data = new Odometry(null);
    // Deserialize message field [source_time]
    data.source_time = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [target_time]
    data.target_time = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [sensor_F_source_T_target]
    data.sensor_F_source_T_target = geometry_msgs.msg.PoseWithCovariance.deserialize(buffer, bufferOffset);
    // Deserialize message field [body_F_source_T_target]
    data.body_F_source_T_target = geometry_msgs.msg.PoseWithCovariance.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 704;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/Odometry';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '422b88b6dc476361c3b1485e5b6113f4';
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Odometry(null);
    if (msg.source_time !== undefined) {
      resolved.source_time = msg.source_time;
    }
    else {
      resolved.source_time = {secs: 0, nsecs: 0}
    }

    if (msg.target_time !== undefined) {
      resolved.target_time = msg.target_time;
    }
    else {
      resolved.target_time = {secs: 0, nsecs: 0}
    }

    if (msg.sensor_F_source_T_target !== undefined) {
      resolved.sensor_F_source_T_target = geometry_msgs.msg.PoseWithCovariance.Resolve(msg.sensor_F_source_T_target)
    }
    else {
      resolved.sensor_F_source_T_target = new geometry_msgs.msg.PoseWithCovariance()
    }

    if (msg.body_F_source_T_target !== undefined) {
      resolved.body_F_source_T_target = geometry_msgs.msg.PoseWithCovariance.Resolve(msg.body_F_source_T_target)
    }
    else {
      resolved.body_F_source_T_target = new geometry_msgs.msg.PoseWithCovariance()
    }

    return resolved;
    }
};

module.exports = Odometry;
