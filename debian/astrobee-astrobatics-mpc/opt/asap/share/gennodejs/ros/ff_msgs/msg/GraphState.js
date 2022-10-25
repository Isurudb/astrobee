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
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class GraphState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.child_frame_id = null;
      this.pose = null;
      this.velocity = null;
      this.gyro_bias = null;
      this.accel_bias = null;
      this.cov_diag = null;
      this.confidence = null;
      this.num_detected_of_features = null;
      this.num_detected_ar_features = null;
      this.num_detected_ml_features = null;
      this.iterations = null;
      this.optimization_time = null;
      this.update_time = null;
      this.callbacks_time = null;
      this.nodelet_runtime = null;
      this.num_factors = null;
      this.num_of_factors = null;
      this.num_ml_projection_factors = null;
      this.num_ml_pose_factors = null;
      this.num_states = null;
      this.standstill = null;
      this.estimating_bias = null;
      this.fan_speed_mode = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('child_frame_id')) {
        this.child_frame_id = initObj.child_frame_id
      }
      else {
        this.child_frame_id = '';
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('gyro_bias')) {
        this.gyro_bias = initObj.gyro_bias
      }
      else {
        this.gyro_bias = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('accel_bias')) {
        this.accel_bias = initObj.accel_bias
      }
      else {
        this.accel_bias = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('cov_diag')) {
        this.cov_diag = initObj.cov_diag
      }
      else {
        this.cov_diag = new Array(15).fill(0);
      }
      if (initObj.hasOwnProperty('confidence')) {
        this.confidence = initObj.confidence
      }
      else {
        this.confidence = 0;
      }
      if (initObj.hasOwnProperty('num_detected_of_features')) {
        this.num_detected_of_features = initObj.num_detected_of_features
      }
      else {
        this.num_detected_of_features = 0;
      }
      if (initObj.hasOwnProperty('num_detected_ar_features')) {
        this.num_detected_ar_features = initObj.num_detected_ar_features
      }
      else {
        this.num_detected_ar_features = 0;
      }
      if (initObj.hasOwnProperty('num_detected_ml_features')) {
        this.num_detected_ml_features = initObj.num_detected_ml_features
      }
      else {
        this.num_detected_ml_features = 0;
      }
      if (initObj.hasOwnProperty('iterations')) {
        this.iterations = initObj.iterations
      }
      else {
        this.iterations = 0;
      }
      if (initObj.hasOwnProperty('optimization_time')) {
        this.optimization_time = initObj.optimization_time
      }
      else {
        this.optimization_time = 0.0;
      }
      if (initObj.hasOwnProperty('update_time')) {
        this.update_time = initObj.update_time
      }
      else {
        this.update_time = 0.0;
      }
      if (initObj.hasOwnProperty('callbacks_time')) {
        this.callbacks_time = initObj.callbacks_time
      }
      else {
        this.callbacks_time = 0.0;
      }
      if (initObj.hasOwnProperty('nodelet_runtime')) {
        this.nodelet_runtime = initObj.nodelet_runtime
      }
      else {
        this.nodelet_runtime = 0.0;
      }
      if (initObj.hasOwnProperty('num_factors')) {
        this.num_factors = initObj.num_factors
      }
      else {
        this.num_factors = 0;
      }
      if (initObj.hasOwnProperty('num_of_factors')) {
        this.num_of_factors = initObj.num_of_factors
      }
      else {
        this.num_of_factors = 0;
      }
      if (initObj.hasOwnProperty('num_ml_projection_factors')) {
        this.num_ml_projection_factors = initObj.num_ml_projection_factors
      }
      else {
        this.num_ml_projection_factors = 0;
      }
      if (initObj.hasOwnProperty('num_ml_pose_factors')) {
        this.num_ml_pose_factors = initObj.num_ml_pose_factors
      }
      else {
        this.num_ml_pose_factors = 0;
      }
      if (initObj.hasOwnProperty('num_states')) {
        this.num_states = initObj.num_states
      }
      else {
        this.num_states = 0;
      }
      if (initObj.hasOwnProperty('standstill')) {
        this.standstill = initObj.standstill
      }
      else {
        this.standstill = false;
      }
      if (initObj.hasOwnProperty('estimating_bias')) {
        this.estimating_bias = initObj.estimating_bias
      }
      else {
        this.estimating_bias = false;
      }
      if (initObj.hasOwnProperty('fan_speed_mode')) {
        this.fan_speed_mode = initObj.fan_speed_mode
      }
      else {
        this.fan_speed_mode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GraphState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [child_frame_id]
    bufferOffset = _serializer.string(obj.child_frame_id, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.velocity, buffer, bufferOffset);
    // Serialize message field [gyro_bias]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.gyro_bias, buffer, bufferOffset);
    // Serialize message field [accel_bias]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.accel_bias, buffer, bufferOffset);
    // Check that the constant length array field [cov_diag] has the right length
    if (obj.cov_diag.length !== 15) {
      throw new Error('Unable to serialize array field cov_diag - length must be 15')
    }
    // Serialize message field [cov_diag]
    bufferOffset = _arraySerializer.float32(obj.cov_diag, buffer, bufferOffset, 15);
    // Serialize message field [confidence]
    bufferOffset = _serializer.uint8(obj.confidence, buffer, bufferOffset);
    // Serialize message field [num_detected_of_features]
    bufferOffset = _serializer.uint32(obj.num_detected_of_features, buffer, bufferOffset);
    // Serialize message field [num_detected_ar_features]
    bufferOffset = _serializer.uint32(obj.num_detected_ar_features, buffer, bufferOffset);
    // Serialize message field [num_detected_ml_features]
    bufferOffset = _serializer.uint32(obj.num_detected_ml_features, buffer, bufferOffset);
    // Serialize message field [iterations]
    bufferOffset = _serializer.uint32(obj.iterations, buffer, bufferOffset);
    // Serialize message field [optimization_time]
    bufferOffset = _serializer.float32(obj.optimization_time, buffer, bufferOffset);
    // Serialize message field [update_time]
    bufferOffset = _serializer.float32(obj.update_time, buffer, bufferOffset);
    // Serialize message field [callbacks_time]
    bufferOffset = _serializer.float32(obj.callbacks_time, buffer, bufferOffset);
    // Serialize message field [nodelet_runtime]
    bufferOffset = _serializer.float32(obj.nodelet_runtime, buffer, bufferOffset);
    // Serialize message field [num_factors]
    bufferOffset = _serializer.uint32(obj.num_factors, buffer, bufferOffset);
    // Serialize message field [num_of_factors]
    bufferOffset = _serializer.uint32(obj.num_of_factors, buffer, bufferOffset);
    // Serialize message field [num_ml_projection_factors]
    bufferOffset = _serializer.uint32(obj.num_ml_projection_factors, buffer, bufferOffset);
    // Serialize message field [num_ml_pose_factors]
    bufferOffset = _serializer.uint32(obj.num_ml_pose_factors, buffer, bufferOffset);
    // Serialize message field [num_states]
    bufferOffset = _serializer.uint32(obj.num_states, buffer, bufferOffset);
    // Serialize message field [standstill]
    bufferOffset = _serializer.bool(obj.standstill, buffer, bufferOffset);
    // Serialize message field [estimating_bias]
    bufferOffset = _serializer.bool(obj.estimating_bias, buffer, bufferOffset);
    // Serialize message field [fan_speed_mode]
    bufferOffset = _serializer.uint8(obj.fan_speed_mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GraphState
    let len;
    let data = new GraphState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [child_frame_id]
    data.child_frame_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [gyro_bias]
    data.gyro_bias = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [accel_bias]
    data.accel_bias = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [cov_diag]
    data.cov_diag = _arrayDeserializer.float32(buffer, bufferOffset, 15)
    // Deserialize message field [confidence]
    data.confidence = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [num_detected_of_features]
    data.num_detected_of_features = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [num_detected_ar_features]
    data.num_detected_ar_features = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [num_detected_ml_features]
    data.num_detected_ml_features = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [iterations]
    data.iterations = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [optimization_time]
    data.optimization_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [update_time]
    data.update_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [callbacks_time]
    data.callbacks_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [nodelet_runtime]
    data.nodelet_runtime = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [num_factors]
    data.num_factors = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [num_of_factors]
    data.num_of_factors = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [num_ml_projection_factors]
    data.num_ml_projection_factors = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [num_ml_pose_factors]
    data.num_ml_pose_factors = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [num_states]
    data.num_states = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [standstill]
    data.standstill = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [estimating_bias]
    data.estimating_bias = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [fan_speed_mode]
    data.fan_speed_mode = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.child_frame_id.length;
    return length + 248;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/GraphState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd0020fbc20fe81214e0f3f2b41dd4c22';
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
    
    std_msgs/Header header # header with timestamp
    string child_frame_id # frame ID
    # State Estimates
    geometry_msgs/Pose pose # world_T_body 
    geometry_msgs/Vector3 velocity # body velocity
    geometry_msgs/Vector3 gyro_bias # estimated gyro bias
    geometry_msgs/Vector3 accel_bias # estimated accel bias
    # Covariances/Confidences
    # covariance diagonal. 1-3 orientation, 4-6 gyro bias, 7-9 velocity, 10-12 accel bias, 13-15 position
    float32[15] cov_diag
    # confidence in estimate. 0 is good, 1 is a bit confused, 2 is lost
    uint8 confidence
    uint8 CONFIDENCE_GOOD = 0	# Tracking well
    uint8 CONFIDENCE_POOR = 1	# Tracking poorly
    uint8 CONFIDENCE_LOST = 2	# We are lost
    # Stats
    uint32 num_detected_of_features  
    uint32 num_detected_ar_features 
    uint32 num_detected_ml_features 
    uint32 iterations # Optimization iterations
    float32 optimization_time
    float32 update_time # Include optimization_time and other operations to add data to graph
    float32 callbacks_time # Includes processing msgs and their callbacks
    float32 nodelet_runtime # Total runtime of nodelet iteration.  Includes update and callback time
    uint32 num_factors
    uint32 num_of_factors
    uint32 num_ml_projection_factors
    uint32 num_ml_pose_factors
    uint32 num_states
    # Status
    bool standstill
    bool estimating_bias # Are we busy estimating the bias?
    uint8 fan_speed_mode # Used for imu filtering
    
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
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
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
    const resolved = new GraphState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.child_frame_id !== undefined) {
      resolved.child_frame_id = msg.child_frame_id;
    }
    else {
      resolved.child_frame_id = ''
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.Pose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.Pose()
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = geometry_msgs.msg.Vector3.Resolve(msg.velocity)
    }
    else {
      resolved.velocity = new geometry_msgs.msg.Vector3()
    }

    if (msg.gyro_bias !== undefined) {
      resolved.gyro_bias = geometry_msgs.msg.Vector3.Resolve(msg.gyro_bias)
    }
    else {
      resolved.gyro_bias = new geometry_msgs.msg.Vector3()
    }

    if (msg.accel_bias !== undefined) {
      resolved.accel_bias = geometry_msgs.msg.Vector3.Resolve(msg.accel_bias)
    }
    else {
      resolved.accel_bias = new geometry_msgs.msg.Vector3()
    }

    if (msg.cov_diag !== undefined) {
      resolved.cov_diag = msg.cov_diag;
    }
    else {
      resolved.cov_diag = new Array(15).fill(0)
    }

    if (msg.confidence !== undefined) {
      resolved.confidence = msg.confidence;
    }
    else {
      resolved.confidence = 0
    }

    if (msg.num_detected_of_features !== undefined) {
      resolved.num_detected_of_features = msg.num_detected_of_features;
    }
    else {
      resolved.num_detected_of_features = 0
    }

    if (msg.num_detected_ar_features !== undefined) {
      resolved.num_detected_ar_features = msg.num_detected_ar_features;
    }
    else {
      resolved.num_detected_ar_features = 0
    }

    if (msg.num_detected_ml_features !== undefined) {
      resolved.num_detected_ml_features = msg.num_detected_ml_features;
    }
    else {
      resolved.num_detected_ml_features = 0
    }

    if (msg.iterations !== undefined) {
      resolved.iterations = msg.iterations;
    }
    else {
      resolved.iterations = 0
    }

    if (msg.optimization_time !== undefined) {
      resolved.optimization_time = msg.optimization_time;
    }
    else {
      resolved.optimization_time = 0.0
    }

    if (msg.update_time !== undefined) {
      resolved.update_time = msg.update_time;
    }
    else {
      resolved.update_time = 0.0
    }

    if (msg.callbacks_time !== undefined) {
      resolved.callbacks_time = msg.callbacks_time;
    }
    else {
      resolved.callbacks_time = 0.0
    }

    if (msg.nodelet_runtime !== undefined) {
      resolved.nodelet_runtime = msg.nodelet_runtime;
    }
    else {
      resolved.nodelet_runtime = 0.0
    }

    if (msg.num_factors !== undefined) {
      resolved.num_factors = msg.num_factors;
    }
    else {
      resolved.num_factors = 0
    }

    if (msg.num_of_factors !== undefined) {
      resolved.num_of_factors = msg.num_of_factors;
    }
    else {
      resolved.num_of_factors = 0
    }

    if (msg.num_ml_projection_factors !== undefined) {
      resolved.num_ml_projection_factors = msg.num_ml_projection_factors;
    }
    else {
      resolved.num_ml_projection_factors = 0
    }

    if (msg.num_ml_pose_factors !== undefined) {
      resolved.num_ml_pose_factors = msg.num_ml_pose_factors;
    }
    else {
      resolved.num_ml_pose_factors = 0
    }

    if (msg.num_states !== undefined) {
      resolved.num_states = msg.num_states;
    }
    else {
      resolved.num_states = 0
    }

    if (msg.standstill !== undefined) {
      resolved.standstill = msg.standstill;
    }
    else {
      resolved.standstill = false
    }

    if (msg.estimating_bias !== undefined) {
      resolved.estimating_bias = msg.estimating_bias;
    }
    else {
      resolved.estimating_bias = false
    }

    if (msg.fan_speed_mode !== undefined) {
      resolved.fan_speed_mode = msg.fan_speed_mode;
    }
    else {
      resolved.fan_speed_mode = 0
    }

    return resolved;
    }
};

// Constants for message
GraphState.Constants = {
  CONFIDENCE_GOOD: 0,
  CONFIDENCE_POOR: 1,
  CONFIDENCE_LOST: 2,
}

module.exports = GraphState;
