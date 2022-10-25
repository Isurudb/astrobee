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

class FamCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.wrench = null;
      this.accel = null;
      this.alpha = null;
      this.status = null;
      this.position_error = null;
      this.position_error_integrated = null;
      this.attitude_error = null;
      this.attitude_error_integrated = null;
      this.attitude_error_mag = null;
      this.control_mode = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('wrench')) {
        this.wrench = initObj.wrench
      }
      else {
        this.wrench = new geometry_msgs.msg.Wrench();
      }
      if (initObj.hasOwnProperty('accel')) {
        this.accel = initObj.accel
      }
      else {
        this.accel = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('alpha')) {
        this.alpha = initObj.alpha
      }
      else {
        this.alpha = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = 0;
      }
      if (initObj.hasOwnProperty('position_error')) {
        this.position_error = initObj.position_error
      }
      else {
        this.position_error = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('position_error_integrated')) {
        this.position_error_integrated = initObj.position_error_integrated
      }
      else {
        this.position_error_integrated = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('attitude_error')) {
        this.attitude_error = initObj.attitude_error
      }
      else {
        this.attitude_error = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('attitude_error_integrated')) {
        this.attitude_error_integrated = initObj.attitude_error_integrated
      }
      else {
        this.attitude_error_integrated = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('attitude_error_mag')) {
        this.attitude_error_mag = initObj.attitude_error_mag
      }
      else {
        this.attitude_error_mag = 0.0;
      }
      if (initObj.hasOwnProperty('control_mode')) {
        this.control_mode = initObj.control_mode
      }
      else {
        this.control_mode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FamCommand
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [wrench]
    bufferOffset = geometry_msgs.msg.Wrench.serialize(obj.wrench, buffer, bufferOffset);
    // Serialize message field [accel]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.accel, buffer, bufferOffset);
    // Serialize message field [alpha]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.alpha, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.uint8(obj.status, buffer, bufferOffset);
    // Serialize message field [position_error]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.position_error, buffer, bufferOffset);
    // Serialize message field [position_error_integrated]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.position_error_integrated, buffer, bufferOffset);
    // Serialize message field [attitude_error]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.attitude_error, buffer, bufferOffset);
    // Serialize message field [attitude_error_integrated]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.attitude_error_integrated, buffer, bufferOffset);
    // Serialize message field [attitude_error_mag]
    bufferOffset = _serializer.float32(obj.attitude_error_mag, buffer, bufferOffset);
    // Serialize message field [control_mode]
    bufferOffset = _serializer.uint8(obj.control_mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FamCommand
    let len;
    let data = new FamCommand(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [wrench]
    data.wrench = geometry_msgs.msg.Wrench.deserialize(buffer, bufferOffset);
    // Deserialize message field [accel]
    data.accel = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [alpha]
    data.alpha = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [position_error]
    data.position_error = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [position_error_integrated]
    data.position_error_integrated = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [attitude_error]
    data.attitude_error = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [attitude_error_integrated]
    data.attitude_error_integrated = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [attitude_error_mag]
    data.attitude_error_mag = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [control_mode]
    data.control_mode = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 198;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/FamCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'baf174131dee1a8b03d9d5feac8aa809';
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
    # Command sent from control to the FAM.
    
    std_msgs/Header header # header with time stamp
    
    # force and torque
    geometry_msgs/Wrench wrench
    # linear acceleration (wrench w/out estimated mass)
    geometry_msgs/Vector3 accel
    # angular accceleration (wrench w/out estimated mass)
    geometry_msgs/Vector3 alpha
    
    # status byte from GNC ICD
    uint8 status
    
    # position error
    geometry_msgs/Vector3 position_error
    # integrated position error
    geometry_msgs/Vector3 position_error_integrated
    
    # attitude error
    geometry_msgs/Vector3 attitude_error
    # integrated attitude error
    geometry_msgs/Vector3 attitude_error_integrated
    # magnitude of attitude error
    float32 attitude_error_mag
    
    # control mode from GNC ICD
    uint8 control_mode
    
    
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
    MSG: geometry_msgs/Wrench
    # This represents force in free space, separated into
    # its linear and angular parts.
    Vector3  force
    Vector3  torque
    
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
    const resolved = new FamCommand(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.wrench !== undefined) {
      resolved.wrench = geometry_msgs.msg.Wrench.Resolve(msg.wrench)
    }
    else {
      resolved.wrench = new geometry_msgs.msg.Wrench()
    }

    if (msg.accel !== undefined) {
      resolved.accel = geometry_msgs.msg.Vector3.Resolve(msg.accel)
    }
    else {
      resolved.accel = new geometry_msgs.msg.Vector3()
    }

    if (msg.alpha !== undefined) {
      resolved.alpha = geometry_msgs.msg.Vector3.Resolve(msg.alpha)
    }
    else {
      resolved.alpha = new geometry_msgs.msg.Vector3()
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = 0
    }

    if (msg.position_error !== undefined) {
      resolved.position_error = geometry_msgs.msg.Vector3.Resolve(msg.position_error)
    }
    else {
      resolved.position_error = new geometry_msgs.msg.Vector3()
    }

    if (msg.position_error_integrated !== undefined) {
      resolved.position_error_integrated = geometry_msgs.msg.Vector3.Resolve(msg.position_error_integrated)
    }
    else {
      resolved.position_error_integrated = new geometry_msgs.msg.Vector3()
    }

    if (msg.attitude_error !== undefined) {
      resolved.attitude_error = geometry_msgs.msg.Vector3.Resolve(msg.attitude_error)
    }
    else {
      resolved.attitude_error = new geometry_msgs.msg.Vector3()
    }

    if (msg.attitude_error_integrated !== undefined) {
      resolved.attitude_error_integrated = geometry_msgs.msg.Vector3.Resolve(msg.attitude_error_integrated)
    }
    else {
      resolved.attitude_error_integrated = new geometry_msgs.msg.Vector3()
    }

    if (msg.attitude_error_mag !== undefined) {
      resolved.attitude_error_mag = msg.attitude_error_mag;
    }
    else {
      resolved.attitude_error_mag = 0.0
    }

    if (msg.control_mode !== undefined) {
      resolved.control_mode = msg.control_mode;
    }
    else {
      resolved.control_mode = 0
    }

    return resolved;
    }
};

module.exports = FamCommand;
