;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::DockAction)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'DockAction (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::DOCKACTION")
  (make-package "FF_MSGS::DOCKACTION"))

(in-package "ROS")
;;//! \htmlinclude DockAction.msg.html


(defclass ff_msgs::DockAction
  :super ros::object
  :slots (_action_goal _action_result _action_feedback ))

(defmethod ff_msgs::DockAction
  (:init
   (&key
    ((:action_goal __action_goal) (instance ff_msgs::DockActionGoal :init))
    ((:action_result __action_result) (instance ff_msgs::DockActionResult :init))
    ((:action_feedback __action_feedback) (instance ff_msgs::DockActionFeedback :init))
    )
   (send-super :init)
   (setq _action_goal __action_goal)
   (setq _action_result __action_result)
   (setq _action_feedback __action_feedback)
   self)
  (:action_goal
   (&rest __action_goal)
   (if (keywordp (car __action_goal))
       (send* _action_goal __action_goal)
     (progn
       (if __action_goal (setq _action_goal (car __action_goal)))
       _action_goal)))
  (:action_result
   (&rest __action_result)
   (if (keywordp (car __action_result))
       (send* _action_result __action_result)
     (progn
       (if __action_result (setq _action_result (car __action_result)))
       _action_result)))
  (:action_feedback
   (&rest __action_feedback)
   (if (keywordp (car __action_feedback))
       (send* _action_feedback __action_feedback)
     (progn
       (if __action_feedback (setq _action_feedback (car __action_feedback)))
       _action_feedback)))
  (:serialization-length
   ()
   (+
    ;; ff_msgs/DockActionGoal _action_goal
    (send _action_goal :serialization-length)
    ;; ff_msgs/DockActionResult _action_result
    (send _action_result :serialization-length)
    ;; ff_msgs/DockActionFeedback _action_feedback
    (send _action_feedback :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; ff_msgs/DockActionGoal _action_goal
       (send _action_goal :serialize s)
     ;; ff_msgs/DockActionResult _action_result
       (send _action_result :serialize s)
     ;; ff_msgs/DockActionFeedback _action_feedback
       (send _action_feedback :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; ff_msgs/DockActionGoal _action_goal
     (send _action_goal :deserialize buf ptr-) (incf ptr- (send _action_goal :serialization-length))
   ;; ff_msgs/DockActionResult _action_result
     (send _action_result :deserialize buf ptr-) (incf ptr- (send _action_result :serialization-length))
   ;; ff_msgs/DockActionFeedback _action_feedback
     (send _action_feedback :deserialize buf ptr-) (incf ptr- (send _action_feedback :serialization-length))
   ;;
   self)
  )

(setf (get ff_msgs::DockAction :md5sum-) "e5850d2946aabf5acd2b50c016179109")
(setf (get ff_msgs::DockAction :datatype-) "ff_msgs/DockAction")
(setf (get ff_msgs::DockAction :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

DockActionGoal action_goal
DockActionResult action_result
DockActionFeedback action_feedback

================================================================================
MSG: ff_msgs/DockActionGoal
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

Header header
actionlib_msgs/GoalID goal_id
DockGoal goal

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
MSG: ff_msgs/DockGoal
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
# 
# All rights reserved.
# 
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the \"License\"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.
#
# Dock action for the Docking node

# Do we want to dock or undock
uint8 command
uint8 DOCK    = 1
uint8 UNDOCK  = 2

# Which berth (values must match)
uint8 berth
uint8 BERTH_UNKNOWN = 0 # When we undock, we don't know what berth we are in
uint8 BERTH_1       = 1
uint8 BERTH_2       = 2

# Return to dock
bool return_dock


================================================================================
MSG: ff_msgs/DockActionResult
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

Header header
actionlib_msgs/GoalStatus status
DockResult result

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
MSG: ff_msgs/DockResult
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

# Result
int32 response
int32 CANCELLED                          = 5
int32 ALREADY_DOCKED                     = 4
int32 ALREADY_UNDOCKED                   = 3
int32 UNDOCKED                           = 2
int32 DOCKED                             = 1
int32 PREEMPTED                          = 0
int32 INVALID_COMMAND                    = -1
int32 INVALID_BERTH                      = -2
int32 NOT_IN_UNDOCKED_STATE              = -3
int32 NOT_IN_DOCKED_STATE                = -4
int32 SWITCH_TO_ML_FAILED                = -5
int32 SWITCH_TO_AR_FAILED                = -6
int32 SWITCH_TO_NO_FAILED                = -7
int32 PREP_DISABLE_FAILED                = -8
int32 PREP_ENABLE_FAILED                 = -9
int32 MOTION_APPROACH_FAILED             = -10
int32 MOTION_COMPLETE_FAILED             = -11
int32 MOTION_ATTACHED_FAILED             = -12
int32 EPS_UNDOCK_FAILED                  = -13
int32 EPS_DOCK_FAILED                    = -14
int32 TOO_FAR_AWAY_FROM_APPROACH         = -15

# Human readable FSM result for debugging
string fsm_result


================================================================================
MSG: ff_msgs/DockActionFeedback
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

Header header
actionlib_msgs/GoalStatus status
DockFeedback feedback

================================================================================
MSG: ff_msgs/DockFeedback
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

# Feedback
ff_msgs/DockState state


================================================================================
MSG: ff_msgs/DockState
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
# 
# All rights reserved.
# 
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the \"License\"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.
#
# Response for Dock/Undock goals

# Header with timestamp
std_msgs/Header header

# Feedback
int8 state
int8 RECOVERY_SWITCHING_TO_ML_LOC       = 15
int8 RECOVERY_MOVING_TO_APPROACH_POSE   = 14
int8 RECOVERY_WAITING_FOR_SPIN_DOWN     = 13
int8 RECOVERY_SWITCHING_TO_NO_LOC       = 12
int8 INITIALIZING                       = 11
int8 UNKNOWN                            = 10
int8 DOCKING_MAX_STATE                  = 7
int8 DOCKING_SWITCHING_TO_ML_LOC        = 7
int8 DOCKING_MOVING_TO_APPROACH_POSE    = 6
int8 DOCKING_SWITCHING_TO_AR_LOC        = 5
int8 DOCKING_MOVING_TO_COMPLETE_POSE    = 4
int8 DOCKING_CHECKING_ATTACHED          = 3
int8 DOCKING_WAITING_FOR_SPIN_DOWN      = 2
int8 DOCKING_SWITCHING_TO_NO_LOC        = 1
int8 DOCKED                             = 0
int8 UNDOCKING_SWITCHING_TO_ML_LOC      = -1
int8 UNDOCKING_WAITING_FOR_SPIN_UP      = -2
int8 UNDOCKING_MOVING_TO_APPROACH_POSE  = -3
int8 UNDOCKED                           = -4
int8 UNDOCKING_MAX_STATE                = -4

# A human readble version of the (event) -> [state] transition
string fsm_event
string fsm_state
")



(provide :ff_msgs/DockAction "e5850d2946aabf5acd2b50c016179109")

