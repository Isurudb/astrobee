// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let DepthLandmark = require('./DepthLandmark.js');
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class DepthLandmarks {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.camera_id = null;
      this.end_seen = null;
      this.update_global_pose = null;
      this.sensor_T_handrail = null;
      this.sensor_t_line_points = null;
      this.sensor_t_line_endpoints = null;
      this.sensor_t_plane_points = null;
      this.landmarks = null;
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
      if (initObj.hasOwnProperty('end_seen')) {
        this.end_seen = initObj.end_seen
      }
      else {
        this.end_seen = 0;
      }
      if (initObj.hasOwnProperty('update_global_pose')) {
        this.update_global_pose = initObj.update_global_pose
      }
      else {
        this.update_global_pose = 0;
      }
      if (initObj.hasOwnProperty('sensor_T_handrail')) {
        this.sensor_T_handrail = initObj.sensor_T_handrail
      }
      else {
        this.sensor_T_handrail = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('sensor_t_line_points')) {
        this.sensor_t_line_points = initObj.sensor_t_line_points
      }
      else {
        this.sensor_t_line_points = [];
      }
      if (initObj.hasOwnProperty('sensor_t_line_endpoints')) {
        this.sensor_t_line_endpoints = initObj.sensor_t_line_endpoints
      }
      else {
        this.sensor_t_line_endpoints = [];
      }
      if (initObj.hasOwnProperty('sensor_t_plane_points')) {
        this.sensor_t_plane_points = initObj.sensor_t_plane_points
      }
      else {
        this.sensor_t_plane_points = [];
      }
      if (initObj.hasOwnProperty('landmarks')) {
        this.landmarks = initObj.landmarks
      }
      else {
        this.landmarks = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DepthLandmarks
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [camera_id]
    bufferOffset = _serializer.uint32(obj.camera_id, buffer, bufferOffset);
    // Serialize message field [end_seen]
    bufferOffset = _serializer.uint8(obj.end_seen, buffer, bufferOffset);
    // Serialize message field [update_global_pose]
    bufferOffset = _serializer.uint8(obj.update_global_pose, buffer, bufferOffset);
    // Serialize message field [sensor_T_handrail]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.sensor_T_handrail, buffer, bufferOffset);
    // Serialize message field [sensor_t_line_points]
    // Serialize the length for message field [sensor_t_line_points]
    bufferOffset = _serializer.uint32(obj.sensor_t_line_points.length, buffer, bufferOffset);
    obj.sensor_t_line_points.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point32.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [sensor_t_line_endpoints]
    // Serialize the length for message field [sensor_t_line_endpoints]
    bufferOffset = _serializer.uint32(obj.sensor_t_line_endpoints.length, buffer, bufferOffset);
    obj.sensor_t_line_endpoints.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [sensor_t_plane_points]
    // Serialize the length for message field [sensor_t_plane_points]
    bufferOffset = _serializer.uint32(obj.sensor_t_plane_points.length, buffer, bufferOffset);
    obj.sensor_t_plane_points.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point32.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [landmarks]
    // Serialize the length for message field [landmarks]
    bufferOffset = _serializer.uint32(obj.landmarks.length, buffer, bufferOffset);
    obj.landmarks.forEach((val) => {
      bufferOffset = DepthLandmark.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DepthLandmarks
    let len;
    let data = new DepthLandmarks(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [camera_id]
    data.camera_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [end_seen]
    data.end_seen = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [update_global_pose]
    data.update_global_pose = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [sensor_T_handrail]
    data.sensor_T_handrail = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [sensor_t_line_points]
    // Deserialize array length for message field [sensor_t_line_points]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.sensor_t_line_points = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.sensor_t_line_points[i] = geometry_msgs.msg.Point32.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [sensor_t_line_endpoints]
    // Deserialize array length for message field [sensor_t_line_endpoints]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.sensor_t_line_endpoints = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.sensor_t_line_endpoints[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [sensor_t_plane_points]
    // Deserialize array length for message field [sensor_t_plane_points]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.sensor_t_plane_points = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.sensor_t_plane_points[i] = geometry_msgs.msg.Point32.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [landmarks]
    // Deserialize array length for message field [landmarks]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.landmarks = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.landmarks[i] = DepthLandmark.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 12 * object.sensor_t_line_points.length;
    length += 24 * object.sensor_t_line_endpoints.length;
    length += 12 * object.sensor_t_plane_points.length;
    length += 12 * object.landmarks.length;
    return length + 78;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/DepthLandmarks';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7fc86a54f996c15d2798a19b023404dc';
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
    # An observation of a handrail from a depth image.
    
    Header header                                  # Image header, with time stamp
    uint32 camera_id                               # Image ID, associated with registration
    uint8 end_seen                                 # Whether the handrail endpoint was detected
    uint8 update_global_pose                       # Whether to update the global pose
    geometry_msgs/Pose sensor_T_handrail           # Handrail center in the sensor frame
    geometry_msgs/Point32[] sensor_t_line_points   # Detected line points
    geometry_msgs/Point[] sensor_t_line_endpoints  # Detected line endpoints
    geometry_msgs/Point32[] sensor_t_plane_points  # Detected plane points
    ff_msgs/DepthLandmark[] landmarks              # List of landmarks seen TODO(rsoussan): This should be removed
    
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
    MSG: geometry_msgs/Point32
    # This contains the position of a point in free space(with 32 bits of precision).
    # It is recommeded to use Point wherever possible instead of Point32.  
    # 
    # This recommendation is to promote interoperability.  
    #
    # This message is designed to take up less space when sending
    # lots of points at once, as in the case of a PointCloud.  
    
    float32 x
    float32 y
    float32 z
    ================================================================================
    MSG: ff_msgs/DepthLandmark
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
    # A landmark seen from a depth landmark
    
    float32 u     # First coordinate in the image plane
    float32 v     # Second coordinate in the image plane
    float32 w     # Depth
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DepthLandmarks(null);
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

    if (msg.end_seen !== undefined) {
      resolved.end_seen = msg.end_seen;
    }
    else {
      resolved.end_seen = 0
    }

    if (msg.update_global_pose !== undefined) {
      resolved.update_global_pose = msg.update_global_pose;
    }
    else {
      resolved.update_global_pose = 0
    }

    if (msg.sensor_T_handrail !== undefined) {
      resolved.sensor_T_handrail = geometry_msgs.msg.Pose.Resolve(msg.sensor_T_handrail)
    }
    else {
      resolved.sensor_T_handrail = new geometry_msgs.msg.Pose()
    }

    if (msg.sensor_t_line_points !== undefined) {
      resolved.sensor_t_line_points = new Array(msg.sensor_t_line_points.length);
      for (let i = 0; i < resolved.sensor_t_line_points.length; ++i) {
        resolved.sensor_t_line_points[i] = geometry_msgs.msg.Point32.Resolve(msg.sensor_t_line_points[i]);
      }
    }
    else {
      resolved.sensor_t_line_points = []
    }

    if (msg.sensor_t_line_endpoints !== undefined) {
      resolved.sensor_t_line_endpoints = new Array(msg.sensor_t_line_endpoints.length);
      for (let i = 0; i < resolved.sensor_t_line_endpoints.length; ++i) {
        resolved.sensor_t_line_endpoints[i] = geometry_msgs.msg.Point.Resolve(msg.sensor_t_line_endpoints[i]);
      }
    }
    else {
      resolved.sensor_t_line_endpoints = []
    }

    if (msg.sensor_t_plane_points !== undefined) {
      resolved.sensor_t_plane_points = new Array(msg.sensor_t_plane_points.length);
      for (let i = 0; i < resolved.sensor_t_plane_points.length; ++i) {
        resolved.sensor_t_plane_points[i] = geometry_msgs.msg.Point32.Resolve(msg.sensor_t_plane_points[i]);
      }
    }
    else {
      resolved.sensor_t_plane_points = []
    }

    if (msg.landmarks !== undefined) {
      resolved.landmarks = new Array(msg.landmarks.length);
      for (let i = 0; i < resolved.landmarks.length; ++i) {
        resolved.landmarks[i] = DepthLandmark.Resolve(msg.landmarks[i]);
      }
    }
    else {
      resolved.landmarks = []
    }

    return resolved;
    }
};

module.exports = DepthLandmarks;
