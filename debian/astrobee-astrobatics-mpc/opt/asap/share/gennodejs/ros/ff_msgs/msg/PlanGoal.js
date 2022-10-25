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

class PlanGoal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.states = null;
      this.faceforward = null;
      this.check_obstacles = null;
      this.desired_vel = null;
      this.desired_accel = null;
      this.desired_omega = null;
      this.desired_alpha = null;
      this.desired_rate = null;
      this.max_time = null;
    }
    else {
      if (initObj.hasOwnProperty('states')) {
        this.states = initObj.states
      }
      else {
        this.states = [];
      }
      if (initObj.hasOwnProperty('faceforward')) {
        this.faceforward = initObj.faceforward
      }
      else {
        this.faceforward = false;
      }
      if (initObj.hasOwnProperty('check_obstacles')) {
        this.check_obstacles = initObj.check_obstacles
      }
      else {
        this.check_obstacles = false;
      }
      if (initObj.hasOwnProperty('desired_vel')) {
        this.desired_vel = initObj.desired_vel
      }
      else {
        this.desired_vel = 0.0;
      }
      if (initObj.hasOwnProperty('desired_accel')) {
        this.desired_accel = initObj.desired_accel
      }
      else {
        this.desired_accel = 0.0;
      }
      if (initObj.hasOwnProperty('desired_omega')) {
        this.desired_omega = initObj.desired_omega
      }
      else {
        this.desired_omega = 0.0;
      }
      if (initObj.hasOwnProperty('desired_alpha')) {
        this.desired_alpha = initObj.desired_alpha
      }
      else {
        this.desired_alpha = 0.0;
      }
      if (initObj.hasOwnProperty('desired_rate')) {
        this.desired_rate = initObj.desired_rate
      }
      else {
        this.desired_rate = 0.0;
      }
      if (initObj.hasOwnProperty('max_time')) {
        this.max_time = initObj.max_time
      }
      else {
        this.max_time = {secs: 0, nsecs: 0};
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlanGoal
    // Serialize message field [states]
    // Serialize the length for message field [states]
    bufferOffset = _serializer.uint32(obj.states.length, buffer, bufferOffset);
    obj.states.forEach((val) => {
      bufferOffset = geometry_msgs.msg.PoseStamped.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [faceforward]
    bufferOffset = _serializer.bool(obj.faceforward, buffer, bufferOffset);
    // Serialize message field [check_obstacles]
    bufferOffset = _serializer.bool(obj.check_obstacles, buffer, bufferOffset);
    // Serialize message field [desired_vel]
    bufferOffset = _serializer.float32(obj.desired_vel, buffer, bufferOffset);
    // Serialize message field [desired_accel]
    bufferOffset = _serializer.float32(obj.desired_accel, buffer, bufferOffset);
    // Serialize message field [desired_omega]
    bufferOffset = _serializer.float32(obj.desired_omega, buffer, bufferOffset);
    // Serialize message field [desired_alpha]
    bufferOffset = _serializer.float32(obj.desired_alpha, buffer, bufferOffset);
    // Serialize message field [desired_rate]
    bufferOffset = _serializer.float32(obj.desired_rate, buffer, bufferOffset);
    // Serialize message field [max_time]
    bufferOffset = _serializer.duration(obj.max_time, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlanGoal
    let len;
    let data = new PlanGoal(null);
    // Deserialize message field [states]
    // Deserialize array length for message field [states]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.states = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.states[i] = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [faceforward]
    data.faceforward = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [check_obstacles]
    data.check_obstacles = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [desired_vel]
    data.desired_vel = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [desired_accel]
    data.desired_accel = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [desired_omega]
    data.desired_omega = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [desired_alpha]
    data.desired_alpha = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [desired_rate]
    data.desired_rate = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [max_time]
    data.max_time = _deserializer.duration(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.states.forEach((val) => {
      length += geometry_msgs.msg.PoseStamped.getMessageSize(val);
    });
    return length + 34;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/PlanGoal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8a3c023dc4c031730a0eb5ee3812c31e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
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
    # This message describes the PLAN action offered by the PLANNER
    
    geometry_msgs/PoseStamped[] states            # Desired state sequence
    
    bool faceforward                              # Face-forward trajectory?
    bool check_obstacles                          # Check against obstacles?
    
    float32 desired_vel                           # Desired (max) velocity
    float32 desired_accel                         # Desired (max) accel
    float32 desired_omega                         # Desired (max) omega
    float32 desired_alpha                         # Desired (max) alpha
    float32 desired_rate                          # Desired rate
    
    duration max_time                             # Max generation time
    
    
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PlanGoal(null);
    if (msg.states !== undefined) {
      resolved.states = new Array(msg.states.length);
      for (let i = 0; i < resolved.states.length; ++i) {
        resolved.states[i] = geometry_msgs.msg.PoseStamped.Resolve(msg.states[i]);
      }
    }
    else {
      resolved.states = []
    }

    if (msg.faceforward !== undefined) {
      resolved.faceforward = msg.faceforward;
    }
    else {
      resolved.faceforward = false
    }

    if (msg.check_obstacles !== undefined) {
      resolved.check_obstacles = msg.check_obstacles;
    }
    else {
      resolved.check_obstacles = false
    }

    if (msg.desired_vel !== undefined) {
      resolved.desired_vel = msg.desired_vel;
    }
    else {
      resolved.desired_vel = 0.0
    }

    if (msg.desired_accel !== undefined) {
      resolved.desired_accel = msg.desired_accel;
    }
    else {
      resolved.desired_accel = 0.0
    }

    if (msg.desired_omega !== undefined) {
      resolved.desired_omega = msg.desired_omega;
    }
    else {
      resolved.desired_omega = 0.0
    }

    if (msg.desired_alpha !== undefined) {
      resolved.desired_alpha = msg.desired_alpha;
    }
    else {
      resolved.desired_alpha = 0.0
    }

    if (msg.desired_rate !== undefined) {
      resolved.desired_rate = msg.desired_rate;
    }
    else {
      resolved.desired_rate = 0.0
    }

    if (msg.max_time !== undefined) {
      resolved.max_time = msg.max_time;
    }
    else {
      resolved.max_time = {secs: 0, nsecs: 0}
    }

    return resolved;
    }
};

module.exports = PlanGoal;
