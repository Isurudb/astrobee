;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::PerchAction)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'PerchAction (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::PERCHACTION")
  (make-package "FF_MSGS::PERCHACTION"))

(in-package "ROS")
;;//! \htmlinclude PerchAction.msg.html


(defclass ff_msgs::PerchAction
  :super ros::object
  :slots (_action_goal _action_result _action_feedback ))

(defmethod ff_msgs::PerchAction
  (:init
   (&key
    ((:action_goal __action_goal) (instance ff_msgs::PerchActionGoal :init))
    ((:action_result __action_result) (instance ff_msgs::PerchActionResult :init))
    ((:action_feedback __action_feedback) (instance ff_msgs::PerchActionFeedback :init))
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
    ;; ff_msgs/PerchActionGoal _action_goal
    (send _action_goal :serialization-length)
    ;; ff_msgs/PerchActionResult _action_result
    (send _action_result :serialization-length)
    ;; ff_msgs/PerchActionFeedback _action_feedback
    (send _action_feedback :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; ff_msgs/PerchActionGoal _action_goal
       (send _action_goal :serialize s)
     ;; ff_msgs/PerchActionResult _action_result
       (send _action_result :serialize s)
     ;; ff_msgs/PerchActionFeedback _action_feedback
       (send _action_feedback :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; ff_msgs/PerchActionGoal _action_goal
     (send _action_goal :deserialize buf ptr-) (incf ptr- (send _action_goal :serialization-length))
   ;; ff_msgs/PerchActionResult _action_result
     (send _action_result :deserialize buf ptr-) (incf ptr- (send _action_result :serialization-length))
   ;; ff_msgs/PerchActionFeedback _action_feedback
     (send _action_feedback :deserialize buf ptr-) (incf ptr- (send _action_feedback :serialization-length))
   ;;
   self)
  )

(setf (get ff_msgs::PerchAction :md5sum-) "958e3fa9a9d696ad3069d11ffb33960a")
(setf (get ff_msgs::PerchAction :datatype-) "ff_msgs/PerchAction")
(setf (get ff_msgs::PerchAction :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

PerchActionGoal action_goal
PerchActionResult action_result
PerchActionFeedback action_feedback

================================================================================
MSG: ff_msgs/PerchActionGoal
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

Header header
actionlib_msgs/GoalID goal_id
PerchGoal goal

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
MSG: ff_msgs/PerchGoal
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
# This message describes the PERCH action offered by the PERCHER

uint8 command
uint8 PERCH   = 1
uint8 UNPERCH = 2


================================================================================
MSG: ff_msgs/PerchActionResult
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

Header header
actionlib_msgs/GoalStatus status
PerchResult result

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
MSG: ff_msgs/PerchResult
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

# Perch result
int32 response
int32 CANCELLED                          = 5
int32 ALREADY_PERCHED                    = 4
int32 ALREADY_UNPERCHED                  = 3
int32 UNPERCHED                          = 2
int32 PERCHED                            = 1
int32 PREEMPTED                          = 0
int32 INVALID_COMMAND                    = -1
int32 NOT_IN_UNPERCHED_STATE             = -2
int32 NOT_IN_PERCHED_STATE               = -3
int32 SWITCH_FAILED                      = -4
int32 MOTION_FAILED                      = -5
int32 ARM_FAILED                         = -6

# Human readable FSM result for debugging
string fsm_result


================================================================================
MSG: ff_msgs/PerchActionFeedback
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

Header header
actionlib_msgs/GoalStatus status
PerchFeedback feedback

================================================================================
MSG: ff_msgs/PerchFeedback
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

# Feedback
ff_msgs/PerchState state


================================================================================
MSG: ff_msgs/PerchState
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
# The state of the perching system

# Header with timestamp
std_msgs/Header header

# Feedback
int8 state

int8 RECOVERY_MOVING_TO_RECOVERY_POSE   = 18
int8 RECOVERY_SWITCHING_TO_ML_LOC       = 17
int8 RECOVERY_STOWING_ARM               = 16
int8 RECOVERY_MOVING_TO_APPROACH_POSE   = 15
int8 RECOVERY_OPENING_GRIPPER           = 14
int8 INITIALIZING                       = 13
int8 UNKNOWN                            = 12
# Used to check the perching/unperching ranges
int8 PERCHING_MAX_STATE                 = 11
int8 PERCHING_SWITCHING_TO_HR_LOC       = 11
int8 PERCHING_MOVING_TO_APPROACH_POSE   = 10
int8 PERCHING_ENSURING_APPROACH_POSE    = 9
int8 PERCHING_DEPLOYING_ARM             = 8
int8 PERCHING_OPENING_GRIPPER           = 7
int8 PERCHING_MOVING_TO_COMPLETE_POSE   = 6
int8 PERCHING_CLOSING_GRIPPER           = 5
int8 PERCHING_CHECKING_ATTACHED         = 4
int8 PERCHING_WAITING_FOR_SPIN_DOWN     = 3
int8 PERCHING_SWITCHING_TO_PL_LOC       = 2
int8 PERCHING_STOPPING                  = 1
int8 PERCHED                            = 0
int8 UNPERCHING_SWITCHING_TO_HR_LOC     = -1
int8 UNPERCHING_WAITING_FOR_SPIN_UP     = -2
int8 UNPERCHING_OPENING_GRIPPER         = -3
int8 UNPERCHING_MOVING_TO_APPROACH_POSE = -4
int8 UNPERCHING_STOWING_ARM             = -5
int8 UNPERCHING_SWITCHING_TO_ML_LOC     = -6
int8 UNPERCHED                          = -7
# Used to check the perching/unperching ranges
int8 UNPERCHING_MAX_STATE               = -7

# A human readable version of the (event) -> [state] transition
string fsm_event
string fsm_state

")



(provide :ff_msgs/PerchAction "958e3fa9a9d696ad3069d11ffb33960a")

