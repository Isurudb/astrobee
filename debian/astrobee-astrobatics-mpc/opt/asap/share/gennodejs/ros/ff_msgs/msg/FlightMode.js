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

class FlightMode {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.name = null;
      this.control_enabled = null;
      this.tolerance_pos_endpoint = null;
      this.tolerance_pos = null;
      this.tolerance_vel = null;
      this.tolerance_att = null;
      this.tolerance_omega = null;
      this.tolerance_time = null;
      this.att_kp = null;
      this.att_ki = null;
      this.omega_kd = null;
      this.pos_kp = null;
      this.pos_ki = null;
      this.vel_kd = null;
      this.hard_limit_vel = null;
      this.hard_limit_accel = null;
      this.hard_limit_omega = null;
      this.hard_limit_alpha = null;
      this.speed = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('control_enabled')) {
        this.control_enabled = initObj.control_enabled
      }
      else {
        this.control_enabled = false;
      }
      if (initObj.hasOwnProperty('tolerance_pos_endpoint')) {
        this.tolerance_pos_endpoint = initObj.tolerance_pos_endpoint
      }
      else {
        this.tolerance_pos_endpoint = 0.0;
      }
      if (initObj.hasOwnProperty('tolerance_pos')) {
        this.tolerance_pos = initObj.tolerance_pos
      }
      else {
        this.tolerance_pos = 0.0;
      }
      if (initObj.hasOwnProperty('tolerance_vel')) {
        this.tolerance_vel = initObj.tolerance_vel
      }
      else {
        this.tolerance_vel = 0.0;
      }
      if (initObj.hasOwnProperty('tolerance_att')) {
        this.tolerance_att = initObj.tolerance_att
      }
      else {
        this.tolerance_att = 0.0;
      }
      if (initObj.hasOwnProperty('tolerance_omega')) {
        this.tolerance_omega = initObj.tolerance_omega
      }
      else {
        this.tolerance_omega = 0.0;
      }
      if (initObj.hasOwnProperty('tolerance_time')) {
        this.tolerance_time = initObj.tolerance_time
      }
      else {
        this.tolerance_time = 0.0;
      }
      if (initObj.hasOwnProperty('att_kp')) {
        this.att_kp = initObj.att_kp
      }
      else {
        this.att_kp = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('att_ki')) {
        this.att_ki = initObj.att_ki
      }
      else {
        this.att_ki = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('omega_kd')) {
        this.omega_kd = initObj.omega_kd
      }
      else {
        this.omega_kd = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('pos_kp')) {
        this.pos_kp = initObj.pos_kp
      }
      else {
        this.pos_kp = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('pos_ki')) {
        this.pos_ki = initObj.pos_ki
      }
      else {
        this.pos_ki = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('vel_kd')) {
        this.vel_kd = initObj.vel_kd
      }
      else {
        this.vel_kd = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('hard_limit_vel')) {
        this.hard_limit_vel = initObj.hard_limit_vel
      }
      else {
        this.hard_limit_vel = 0.0;
      }
      if (initObj.hasOwnProperty('hard_limit_accel')) {
        this.hard_limit_accel = initObj.hard_limit_accel
      }
      else {
        this.hard_limit_accel = 0.0;
      }
      if (initObj.hasOwnProperty('hard_limit_omega')) {
        this.hard_limit_omega = initObj.hard_limit_omega
      }
      else {
        this.hard_limit_omega = 0.0;
      }
      if (initObj.hasOwnProperty('hard_limit_alpha')) {
        this.hard_limit_alpha = initObj.hard_limit_alpha
      }
      else {
        this.hard_limit_alpha = 0.0;
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FlightMode
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [control_enabled]
    bufferOffset = _serializer.bool(obj.control_enabled, buffer, bufferOffset);
    // Serialize message field [tolerance_pos_endpoint]
    bufferOffset = _serializer.float32(obj.tolerance_pos_endpoint, buffer, bufferOffset);
    // Serialize message field [tolerance_pos]
    bufferOffset = _serializer.float32(obj.tolerance_pos, buffer, bufferOffset);
    // Serialize message field [tolerance_vel]
    bufferOffset = _serializer.float32(obj.tolerance_vel, buffer, bufferOffset);
    // Serialize message field [tolerance_att]
    bufferOffset = _serializer.float32(obj.tolerance_att, buffer, bufferOffset);
    // Serialize message field [tolerance_omega]
    bufferOffset = _serializer.float32(obj.tolerance_omega, buffer, bufferOffset);
    // Serialize message field [tolerance_time]
    bufferOffset = _serializer.float32(obj.tolerance_time, buffer, bufferOffset);
    // Serialize message field [att_kp]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.att_kp, buffer, bufferOffset);
    // Serialize message field [att_ki]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.att_ki, buffer, bufferOffset);
    // Serialize message field [omega_kd]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.omega_kd, buffer, bufferOffset);
    // Serialize message field [pos_kp]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.pos_kp, buffer, bufferOffset);
    // Serialize message field [pos_ki]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.pos_ki, buffer, bufferOffset);
    // Serialize message field [vel_kd]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.vel_kd, buffer, bufferOffset);
    // Serialize message field [hard_limit_vel]
    bufferOffset = _serializer.float32(obj.hard_limit_vel, buffer, bufferOffset);
    // Serialize message field [hard_limit_accel]
    bufferOffset = _serializer.float32(obj.hard_limit_accel, buffer, bufferOffset);
    // Serialize message field [hard_limit_omega]
    bufferOffset = _serializer.float32(obj.hard_limit_omega, buffer, bufferOffset);
    // Serialize message field [hard_limit_alpha]
    bufferOffset = _serializer.float32(obj.hard_limit_alpha, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = _serializer.uint8(obj.speed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FlightMode
    let len;
    let data = new FlightMode(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [control_enabled]
    data.control_enabled = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [tolerance_pos_endpoint]
    data.tolerance_pos_endpoint = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tolerance_pos]
    data.tolerance_pos = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tolerance_vel]
    data.tolerance_vel = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tolerance_att]
    data.tolerance_att = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tolerance_omega]
    data.tolerance_omega = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tolerance_time]
    data.tolerance_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [att_kp]
    data.att_kp = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [att_ki]
    data.att_ki = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [omega_kd]
    data.omega_kd = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [pos_kp]
    data.pos_kp = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [pos_ki]
    data.pos_ki = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [vel_kd]
    data.vel_kd = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [hard_limit_vel]
    data.hard_limit_vel = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [hard_limit_accel]
    data.hard_limit_accel = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [hard_limit_omega]
    data.hard_limit_omega = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [hard_limit_alpha]
    data.hard_limit_alpha = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.name.length;
    return length + 190;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/FlightMode';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0bb389101a5f30087bd644e6596d8e8e';
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
    # This message captures all information in a flight mode
    
    Header header                     # Metadata
    
    string name                       # Name of the flight mode
    
    bool control_enabled              # Is control enabled?
    
    # Tolerances (all in SI units)
    float32 tolerance_pos_endpoint    # Endpoint position tolerance in m
    float32 tolerance_pos             # Position tolerance in m
    float32 tolerance_vel             # Velocity tolerance in m/s
    float32 tolerance_att             # Attitude tolerance in rads
    float32 tolerance_omega           # Angular acceleration tolerance in rad/s
    float32 tolerance_time            # Acceptable lag betwee TX and RX of control
    
    # Controller gains
    geometry_msgs/Vector3 att_kp      # Positional proportional constant
    geometry_msgs/Vector3 att_ki      # Positional integrative constant
    geometry_msgs/Vector3 omega_kd    # Attidue derivative constant
    geometry_msgs/Vector3 pos_kp      # Positional proportional contant
    geometry_msgs/Vector3 pos_ki      # Positional integrative constant
    geometry_msgs/Vector3 vel_kd      # Positional derivative constant
    
    # Hard limit on planning
    float32 hard_limit_vel            # Position tolerance in m/s
    float32 hard_limit_accel          # Position tolerance in m/s^2
    float32 hard_limit_omega          # Position tolerance in rads/s
    float32 hard_limit_alpha          # Position tolerance in rads/s^2
    
    # Impeller speed
    uint8 speed                       # Current speed gain
    uint8 SPEED_MIN        = 0        # Min acceptable gain
    uint8 SPEED_OFF        = 0        # Blowers off
    uint8 SPEED_QUIET      = 1        # Quiet mode
    uint8 SPEED_NOMINAL    = 2        # Nomainal mode
    uint8 SPEED_AGGRESSIVE = 3        # Aggressive mode
    uint8 SPEED_MAX        = 3        # Max acceptable gain
    
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
    const resolved = new FlightMode(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.control_enabled !== undefined) {
      resolved.control_enabled = msg.control_enabled;
    }
    else {
      resolved.control_enabled = false
    }

    if (msg.tolerance_pos_endpoint !== undefined) {
      resolved.tolerance_pos_endpoint = msg.tolerance_pos_endpoint;
    }
    else {
      resolved.tolerance_pos_endpoint = 0.0
    }

    if (msg.tolerance_pos !== undefined) {
      resolved.tolerance_pos = msg.tolerance_pos;
    }
    else {
      resolved.tolerance_pos = 0.0
    }

    if (msg.tolerance_vel !== undefined) {
      resolved.tolerance_vel = msg.tolerance_vel;
    }
    else {
      resolved.tolerance_vel = 0.0
    }

    if (msg.tolerance_att !== undefined) {
      resolved.tolerance_att = msg.tolerance_att;
    }
    else {
      resolved.tolerance_att = 0.0
    }

    if (msg.tolerance_omega !== undefined) {
      resolved.tolerance_omega = msg.tolerance_omega;
    }
    else {
      resolved.tolerance_omega = 0.0
    }

    if (msg.tolerance_time !== undefined) {
      resolved.tolerance_time = msg.tolerance_time;
    }
    else {
      resolved.tolerance_time = 0.0
    }

    if (msg.att_kp !== undefined) {
      resolved.att_kp = geometry_msgs.msg.Vector3.Resolve(msg.att_kp)
    }
    else {
      resolved.att_kp = new geometry_msgs.msg.Vector3()
    }

    if (msg.att_ki !== undefined) {
      resolved.att_ki = geometry_msgs.msg.Vector3.Resolve(msg.att_ki)
    }
    else {
      resolved.att_ki = new geometry_msgs.msg.Vector3()
    }

    if (msg.omega_kd !== undefined) {
      resolved.omega_kd = geometry_msgs.msg.Vector3.Resolve(msg.omega_kd)
    }
    else {
      resolved.omega_kd = new geometry_msgs.msg.Vector3()
    }

    if (msg.pos_kp !== undefined) {
      resolved.pos_kp = geometry_msgs.msg.Vector3.Resolve(msg.pos_kp)
    }
    else {
      resolved.pos_kp = new geometry_msgs.msg.Vector3()
    }

    if (msg.pos_ki !== undefined) {
      resolved.pos_ki = geometry_msgs.msg.Vector3.Resolve(msg.pos_ki)
    }
    else {
      resolved.pos_ki = new geometry_msgs.msg.Vector3()
    }

    if (msg.vel_kd !== undefined) {
      resolved.vel_kd = geometry_msgs.msg.Vector3.Resolve(msg.vel_kd)
    }
    else {
      resolved.vel_kd = new geometry_msgs.msg.Vector3()
    }

    if (msg.hard_limit_vel !== undefined) {
      resolved.hard_limit_vel = msg.hard_limit_vel;
    }
    else {
      resolved.hard_limit_vel = 0.0
    }

    if (msg.hard_limit_accel !== undefined) {
      resolved.hard_limit_accel = msg.hard_limit_accel;
    }
    else {
      resolved.hard_limit_accel = 0.0
    }

    if (msg.hard_limit_omega !== undefined) {
      resolved.hard_limit_omega = msg.hard_limit_omega;
    }
    else {
      resolved.hard_limit_omega = 0.0
    }

    if (msg.hard_limit_alpha !== undefined) {
      resolved.hard_limit_alpha = msg.hard_limit_alpha;
    }
    else {
      resolved.hard_limit_alpha = 0.0
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0
    }

    return resolved;
    }
};

// Constants for message
FlightMode.Constants = {
  SPEED_MIN: 0,
  SPEED_OFF: 0,
  SPEED_QUIET: 1,
  SPEED_NOMINAL: 2,
  SPEED_AGGRESSIVE: 3,
  SPEED_MAX: 3,
}

module.exports = FlightMode;
