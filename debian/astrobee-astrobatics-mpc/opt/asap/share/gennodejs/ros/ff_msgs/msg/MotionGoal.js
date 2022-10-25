// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ControlState = require('./ControlState.js');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class MotionGoal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.command = null;
      this.flight_mode = null;
      this.segment = null;
      this.states = null;
      this.reference_frame = null;
    }
    else {
      if (initObj.hasOwnProperty('command')) {
        this.command = initObj.command
      }
      else {
        this.command = 0;
      }
      if (initObj.hasOwnProperty('flight_mode')) {
        this.flight_mode = initObj.flight_mode
      }
      else {
        this.flight_mode = '';
      }
      if (initObj.hasOwnProperty('segment')) {
        this.segment = initObj.segment
      }
      else {
        this.segment = [];
      }
      if (initObj.hasOwnProperty('states')) {
        this.states = initObj.states
      }
      else {
        this.states = [];
      }
      if (initObj.hasOwnProperty('reference_frame')) {
        this.reference_frame = initObj.reference_frame
      }
      else {
        this.reference_frame = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotionGoal
    // Serialize message field [command]
    bufferOffset = _serializer.uint8(obj.command, buffer, bufferOffset);
    // Serialize message field [flight_mode]
    bufferOffset = _serializer.string(obj.flight_mode, buffer, bufferOffset);
    // Serialize message field [segment]
    // Serialize the length for message field [segment]
    bufferOffset = _serializer.uint32(obj.segment.length, buffer, bufferOffset);
    obj.segment.forEach((val) => {
      bufferOffset = ControlState.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [states]
    // Serialize the length for message field [states]
    bufferOffset = _serializer.uint32(obj.states.length, buffer, bufferOffset);
    obj.states.forEach((val) => {
      bufferOffset = geometry_msgs.msg.PoseStamped.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [reference_frame]
    bufferOffset = _serializer.string(obj.reference_frame, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotionGoal
    let len;
    let data = new MotionGoal(null);
    // Deserialize message field [command]
    data.command = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [flight_mode]
    data.flight_mode = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [segment]
    // Deserialize array length for message field [segment]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.segment = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.segment[i] = ControlState.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [states]
    // Deserialize array length for message field [states]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.states = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.states[i] = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [reference_frame]
    data.reference_frame = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.flight_mode.length;
    length += 160 * object.segment.length;
    object.states.forEach((val) => {
      length += geometry_msgs.msg.PoseStamped.getMessageSize(val);
    });
    length += object.reference_frame.length;
    return length + 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/MotionGoal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1f5494c5016459c2ad2035dda3ef7acb';
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
    # This message describes the MOTION action offered by the CHOREOGRAPHER
    
    # Desired command
    uint8 command
    uint8 STOP        = 0  # Stop immediately
    uint8 IDLE        = 1  # Idle immediately
    uint8 EXEC        = 2  # Execute a given segment
    uint8 MOVE        = 3  # Move through a given set of poses
    uint8 PREP        = 4  # Prepare the system for a given flight mode
    
    # ALL COMMANDS: flight mode. If left empty, the default will be used.
    string flight_mode
    string OFF        = off
    string NOMINAL    = nominal
    string DIFFICULT  = difficult
    string QUIET      = quiet
    
    # These four values no longer used (2020/10). Keep for backward
    # compatibility with archived ISS telemetry bags.
    string PERCHING   = perching
    string UNPERCHING = unperching
    string DOCKING    = docking
    string UNDOCKING  = undocking
    
    string PRECISION  = precision
    
    # EXECUTE ONLY : The segment
    ff_msgs/ControlState[] segment
    
    # MOVE ONLY: Desired state sequence
    geometry_msgs/PoseStamped[] states
    
    string reference_frame
    
    
    ================================================================================
    MSG: ff_msgs/ControlState
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
    # Full state vector containing Time, Pose, Vel, and Accel
    # 
    # when {time}
    # flight_mode {string} - disctates, gains, tolerances, etc.
    # pose {Point position, Quaternion orientation}
    # twist {Vector3 linear, Vector3 angular}
    # accel {Vector3 linear, Vector3 angular}
    
    time when
    geometry_msgs/Pose pose
    geometry_msgs/Twist twist
    geometry_msgs/Twist accel
    
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
    MSG: geometry_msgs/Twist
    # This expresses velocity in free space broken into its linear and angular parts.
    Vector3  linear
    Vector3  angular
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotionGoal(null);
    if (msg.command !== undefined) {
      resolved.command = msg.command;
    }
    else {
      resolved.command = 0
    }

    if (msg.flight_mode !== undefined) {
      resolved.flight_mode = msg.flight_mode;
    }
    else {
      resolved.flight_mode = ''
    }

    if (msg.segment !== undefined) {
      resolved.segment = new Array(msg.segment.length);
      for (let i = 0; i < resolved.segment.length; ++i) {
        resolved.segment[i] = ControlState.Resolve(msg.segment[i]);
      }
    }
    else {
      resolved.segment = []
    }

    if (msg.states !== undefined) {
      resolved.states = new Array(msg.states.length);
      for (let i = 0; i < resolved.states.length; ++i) {
        resolved.states[i] = geometry_msgs.msg.PoseStamped.Resolve(msg.states[i]);
      }
    }
    else {
      resolved.states = []
    }

    if (msg.reference_frame !== undefined) {
      resolved.reference_frame = msg.reference_frame;
    }
    else {
      resolved.reference_frame = ''
    }

    return resolved;
    }
};

// Constants for message
MotionGoal.Constants = {
  STOP: 0,
  IDLE: 1,
  EXEC: 2,
  MOVE: 3,
  PREP: 4,
  OFF: 'off',
  NOMINAL: 'nominal',
  DIFFICULT: 'difficult',
  QUIET: 'quiet',
  PERCHING: 'perching',
  UNPERCHING: 'unperching',
  DOCKING: 'docking',
  UNDOCKING: 'undocking',
  PRECISION: 'precision',
}

module.exports = MotionGoal;
