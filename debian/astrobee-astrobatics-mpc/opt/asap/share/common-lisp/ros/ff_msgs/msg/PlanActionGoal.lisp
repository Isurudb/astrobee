; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude PlanActionGoal.msg.html

(cl:defclass <PlanActionGoal> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (goal_id
    :reader goal_id
    :initarg :goal_id
    :type actionlib_msgs-msg:GoalID
    :initform (cl:make-instance 'actionlib_msgs-msg:GoalID))
   (goal
    :reader goal
    :initarg :goal
    :type ff_msgs-msg:PlanGoal
    :initform (cl:make-instance 'ff_msgs-msg:PlanGoal)))
)

(cl:defclass PlanActionGoal (<PlanActionGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlanActionGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlanActionGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<PlanActionGoal> is deprecated: use ff_msgs-msg:PlanActionGoal instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PlanActionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'goal_id-val :lambda-list '(m))
(cl:defmethod goal_id-val ((m <PlanActionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:goal_id-val is deprecated.  Use ff_msgs-msg:goal_id instead.")
  (goal_id m))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <PlanActionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:goal-val is deprecated.  Use ff_msgs-msg:goal instead.")
  (goal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlanActionGoal>) ostream)
  "Serializes a message object of type '<PlanActionGoal>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal_id) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlanActionGoal>) istream)
  "Deserializes a message object of type '<PlanActionGoal>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal_id) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlanActionGoal>)))
  "Returns string type for a message object of type '<PlanActionGoal>"
  "ff_msgs/PlanActionGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanActionGoal)))
  "Returns string type for a message object of type 'PlanActionGoal"
  "ff_msgs/PlanActionGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlanActionGoal>)))
  "Returns md5sum for a message object of type '<PlanActionGoal>"
  "ef8436f3ef656c529f23027f3bcee781")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlanActionGoal)))
  "Returns md5sum for a message object of type 'PlanActionGoal"
  "ef8436f3ef656c529f23027f3bcee781")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlanActionGoal>)))
  "Returns full string definition for message of type '<PlanActionGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%PlanGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: ff_msgs/PlanGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# This message describes the PLAN action offered by the PLANNER~%~%geometry_msgs/PoseStamped[] states            # Desired state sequence~%~%bool faceforward                              # Face-forward trajectory?~%bool check_obstacles                          # Check against obstacles?~%~%float32 desired_vel                           # Desired (max) velocity~%float32 desired_accel                         # Desired (max) accel~%float32 desired_omega                         # Desired (max) omega~%float32 desired_alpha                         # Desired (max) alpha~%float32 desired_rate                          # Desired rate~%~%duration max_time                             # Max generation time~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlanActionGoal)))
  "Returns full string definition for message of type 'PlanActionGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%PlanGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: ff_msgs/PlanGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# This message describes the PLAN action offered by the PLANNER~%~%geometry_msgs/PoseStamped[] states            # Desired state sequence~%~%bool faceforward                              # Face-forward trajectory?~%bool check_obstacles                          # Check against obstacles?~%~%float32 desired_vel                           # Desired (max) velocity~%float32 desired_accel                         # Desired (max) accel~%float32 desired_omega                         # Desired (max) omega~%float32 desired_alpha                         # Desired (max) alpha~%float32 desired_rate                          # Desired rate~%~%duration max_time                             # Max generation time~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlanActionGoal>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal_id))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlanActionGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'PlanActionGoal
    (cl:cons ':header (header msg))
    (cl:cons ':goal_id (goal_id msg))
    (cl:cons ':goal (goal msg))
))
