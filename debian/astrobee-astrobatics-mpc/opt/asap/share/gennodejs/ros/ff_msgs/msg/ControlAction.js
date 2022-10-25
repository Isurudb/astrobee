// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ControlActionGoal = require('./ControlActionGoal.js');
let ControlActionResult = require('./ControlActionResult.js');
let ControlActionFeedback = require('./ControlActionFeedback.js');

//-----------------------------------------------------------

class ControlAction {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.action_goal = null;
      this.action_result = null;
      this.action_feedback = null;
    }
    else {
      if (initObj.hasOwnProperty('action_goal')) {
        this.action_goal = initObj.action_goal
      }
      else {
        this.action_goal = new ControlActionGoal();
      }
      if (initObj.hasOwnProperty('action_result')) {
        this.action_result = initObj.action_result
      }
      else {
        this.action_result = new ControlActionResult();
      }
      if (initObj.hasOwnProperty('action_feedback')) {
        this.action_feedback = initObj.action_feedback
      }
      else {
        this.action_feedback = new ControlActionFeedback();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControlAction
    // Serialize message field [action_goal]
    bufferOffset = ControlActionGoal.serialize(obj.action_goal, buffer, bufferOffset);
    // Serialize message field [action_result]
    bufferOffset = ControlActionResult.serialize(obj.action_result, buffer, bufferOffset);
    // Serialize message field [action_feedback]
    bufferOffset = ControlActionFeedback.serialize(obj.action_feedback, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControlAction
    let len;
    let data = new ControlAction(null);
    // Deserialize message field [action_goal]
    data.action_goal = ControlActionGoal.deserialize(buffer, bufferOffset);
    // Deserialize message field [action_result]
    data.action_result = ControlActionResult.deserialize(buffer, bufferOffset);
    // Deserialize message field [action_feedback]
    data.action_feedback = ControlActionFeedback.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += ControlActionGoal.getMessageSize(object.action_goal);
    length += ControlActionResult.getMessageSize(object.action_result);
    length += ControlActionFeedback.getMessageSize(object.action_feedback);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/ControlAction';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3503e2680b6b89c99fa60b340caf9072';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    
    ControlActionGoal action_goal
    ControlActionResult action_result
    ControlActionFeedback action_feedback
    
    ================================================================================
    MSG: ff_msgs/ControlActionGoal
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    
    Header header
    actionlib_msgs/GoalID goal_id
    ControlGoal goal
    
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
    MSG: ff_msgs/ControlGoal
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
    # This message describes the CONTROL action offered by GNC::WRAPPER
    
    uint8 command                               # STOP, IDLE, NOMINAL
    uint8 STOP    = 0
    uint8 IDLE    = 1
    uint8 NOMINAL = 2
    
    ff_msgs/ControlState[] segment              # NOMINIAL ONLY: Segment
    
    
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
    MSG: ff_msgs/ControlActionResult
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    
    Header header
    actionlib_msgs/GoalStatus status
    ControlResult result
    
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
    MSG: ff_msgs/ControlResult
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    
    int32 response                              
    int32 SUCCESS                          =  1 # Control succeeded
    int32 PREEMPTED                        =  0 # Control preempted
    int32 CANCELLED                        = -1 # Control cancelled
    int32 CONTROL_FAILED                   = -2 # Control aborted
    int32 EMPTY_SEGMENT                    = -3 # Empty segment in goal
    int32 INVALID_COMMAND                  = -4 # Invalid command in goal
    int32 TIMESYNC_ISSUE                   = -5 # Time begins < -1s in past
    
    ff_msgs/ControlState[] segment              # Final segment flown
    uint32 index                                # Index being processed
    
    
    ================================================================================
    MSG: ff_msgs/ControlActionFeedback
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    
    Header header
    actionlib_msgs/GoalStatus status
    ControlFeedback feedback
    
    ================================================================================
    MSG: ff_msgs/ControlFeedback
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    
    uint32 index                                # Index being processed
    
    ff_msgs/ControlState setpoint               # Current setpoint
    
    float32 error_position                      # Position error
    float32 error_attitude                      # Attitude error
    float32 error_velocity                      # Velocity error
    float32 error_omega                         # Omega error
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControlAction(null);
    if (msg.action_goal !== undefined) {
      resolved.action_goal = ControlActionGoal.Resolve(msg.action_goal)
    }
    else {
      resolved.action_goal = new ControlActionGoal()
    }

    if (msg.action_result !== undefined) {
      resolved.action_result = ControlActionResult.Resolve(msg.action_result)
    }
    else {
      resolved.action_result = new ControlActionResult()
    }

    if (msg.action_feedback !== undefined) {
      resolved.action_feedback = ControlActionFeedback.Resolve(msg.action_feedback)
    }
    else {
      resolved.action_feedback = new ControlActionFeedback()
    }

    return resolved;
    }
};

module.exports = ControlAction;
