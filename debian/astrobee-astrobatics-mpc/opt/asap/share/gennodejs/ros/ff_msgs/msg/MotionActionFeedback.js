// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let MotionFeedback = require('./MotionFeedback.js');
let actionlib_msgs = _finder('actionlib_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class MotionActionFeedback {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.status = null;
      this.feedback = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = new actionlib_msgs.msg.GoalStatus();
      }
      if (initObj.hasOwnProperty('feedback')) {
        this.feedback = initObj.feedback
      }
      else {
        this.feedback = new MotionFeedback();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotionActionFeedback
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = actionlib_msgs.msg.GoalStatus.serialize(obj.status, buffer, bufferOffset);
    // Serialize message field [feedback]
    bufferOffset = MotionFeedback.serialize(obj.feedback, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotionActionFeedback
    let len;
    let data = new MotionActionFeedback(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = actionlib_msgs.msg.GoalStatus.deserialize(buffer, bufferOffset);
    // Deserialize message field [feedback]
    data.feedback = MotionFeedback.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += actionlib_msgs.msg.GoalStatus.getMessageSize(object.status);
    length += MotionFeedback.getMessageSize(object.feedback);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/MotionActionFeedback';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5f91d48e530821eb4c3318d980598eba';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    
    Header header
    actionlib_msgs/GoalStatus status
    MotionFeedback feedback
    
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
    MSG: actionlib_msgs/GoalStatus
    GoalID goal_id
    uint8 status
    uint8 PENDING         = 0   # The goal has yet to be processed by the action server
    uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
    uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                                #   and has since completed its execution (Terminal State)
    uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
    uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                                #    to some failure (Terminal State)
    uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                                #    because the goal was unattainable or invalid (Terminal State)
    uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                                #    and has not yet completed execution
    uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                                #    but the action server has not yet confirmed that the goal is canceled
    uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                                #    and was successfully cancelled (Terminal State)
    uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                                #    sent over the wire by an action server
    
    #Allow for the user to associate a string with GoalStatus for debugging
    string text
    
    
    ================================================================================
    MSG: actionlib_msgs/GoalID
    # The stamp should store the time at which this goal was requested.
    # It is used by an action server when it tries to preempt all
    # goals that were requested before a certain time
    time stamp
    
    # The id provides a way to associate feedback and
    # result message with specific goal requests. The id
    # specified must be unique.
    string id
    
    
    ================================================================================
    MSG: ff_msgs/MotionFeedback
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    
    # The state of the teleop command
    ff_msgs/MotionState state
    
    # Control progress
    ff_msgs/ControlFeedback progress
    
    # Planner progress
    float32 perc_complete
    float32 secs_remaining
    
    
    ================================================================================
    MSG: ff_msgs/MotionState
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
    # Locked topic that registers updates to the internal dock state
    
    # Header with timestamp
    std_msgs/Header header
    
    # The state of the mobility subsystem
    int8 state
    int8 INITIALIZING        = 0
    int8 IDLE                = 1
    int8 STOPPED             = 2
    int8 IDLING              = 3
    int8 STOPPING            = 4
    int8 PREPPING            = 5
    int8 BOOTSTRAPPING       = 6
    int8 PLANNING            = 7
    int8 PREPARING           = 8
    int8 CONTROLLING         = 9
    int8 REPLANNING          = 10
    int8 HALTING             = 11
    int8 REPLAN_WAIT         = 12
    
    # A human readble version of the (event) -> [state] transition
    string fsm_event
    string fsm_state
    
    ================================================================================
    MSG: ff_msgs/ControlFeedback
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    
    uint32 index                                # Index being processed
    
    ff_msgs/ControlState setpoint               # Current setpoint
    
    float32 error_position                      # Position error
    float32 error_attitude                      # Attitude error
    float32 error_velocity                      # Velocity error
    float32 error_omega                         # Omega error
    
    
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotionActionFeedback(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.status !== undefined) {
      resolved.status = actionlib_msgs.msg.GoalStatus.Resolve(msg.status)
    }
    else {
      resolved.status = new actionlib_msgs.msg.GoalStatus()
    }

    if (msg.feedback !== undefined) {
      resolved.feedback = MotionFeedback.Resolve(msg.feedback)
    }
    else {
      resolved.feedback = new MotionFeedback()
    }

    return resolved;
    }
};

module.exports = MotionActionFeedback;
