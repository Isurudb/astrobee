; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude DockFeedback.msg.html

(cl:defclass <DockFeedback> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type ff_msgs-msg:DockState
    :initform (cl:make-instance 'ff_msgs-msg:DockState)))
)

(cl:defclass DockFeedback (<DockFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DockFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DockFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<DockFeedback> is deprecated: use ff_msgs-msg:DockFeedback instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <DockFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:state-val is deprecated.  Use ff_msgs-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DockFeedback>) ostream)
  "Serializes a message object of type '<DockFeedback>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DockFeedback>) istream)
  "Deserializes a message object of type '<DockFeedback>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DockFeedback>)))
  "Returns string type for a message object of type '<DockFeedback>"
  "ff_msgs/DockFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DockFeedback)))
  "Returns string type for a message object of type 'DockFeedback"
  "ff_msgs/DockFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DockFeedback>)))
  "Returns md5sum for a message object of type '<DockFeedback>"
  "ab5cbe2d052a7927a3632f11a64bfa27")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DockFeedback)))
  "Returns md5sum for a message object of type 'DockFeedback"
  "ab5cbe2d052a7927a3632f11a64bfa27")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DockFeedback>)))
  "Returns full string definition for message of type '<DockFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%# Feedback~%ff_msgs/DockState state~%~%~%================================================================================~%MSG: ff_msgs/DockState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Response for Dock/Undock goals~%~%# Header with timestamp~%std_msgs/Header header~%~%# Feedback~%int8 state~%int8 RECOVERY_SWITCHING_TO_ML_LOC       = 15~%int8 RECOVERY_MOVING_TO_APPROACH_POSE   = 14~%int8 RECOVERY_WAITING_FOR_SPIN_DOWN     = 13~%int8 RECOVERY_SWITCHING_TO_NO_LOC       = 12~%int8 INITIALIZING                       = 11~%int8 UNKNOWN                            = 10~%int8 DOCKING_MAX_STATE                  = 7~%int8 DOCKING_SWITCHING_TO_ML_LOC        = 7~%int8 DOCKING_MOVING_TO_APPROACH_POSE    = 6~%int8 DOCKING_SWITCHING_TO_AR_LOC        = 5~%int8 DOCKING_MOVING_TO_COMPLETE_POSE    = 4~%int8 DOCKING_CHECKING_ATTACHED          = 3~%int8 DOCKING_WAITING_FOR_SPIN_DOWN      = 2~%int8 DOCKING_SWITCHING_TO_NO_LOC        = 1~%int8 DOCKED                             = 0~%int8 UNDOCKING_SWITCHING_TO_ML_LOC      = -1~%int8 UNDOCKING_WAITING_FOR_SPIN_UP      = -2~%int8 UNDOCKING_MOVING_TO_APPROACH_POSE  = -3~%int8 UNDOCKED                           = -4~%int8 UNDOCKING_MAX_STATE                = -4~%~%# A human readble version of the (event) -> [state] transition~%string fsm_event~%string fsm_state~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DockFeedback)))
  "Returns full string definition for message of type 'DockFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%# Feedback~%ff_msgs/DockState state~%~%~%================================================================================~%MSG: ff_msgs/DockState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Response for Dock/Undock goals~%~%# Header with timestamp~%std_msgs/Header header~%~%# Feedback~%int8 state~%int8 RECOVERY_SWITCHING_TO_ML_LOC       = 15~%int8 RECOVERY_MOVING_TO_APPROACH_POSE   = 14~%int8 RECOVERY_WAITING_FOR_SPIN_DOWN     = 13~%int8 RECOVERY_SWITCHING_TO_NO_LOC       = 12~%int8 INITIALIZING                       = 11~%int8 UNKNOWN                            = 10~%int8 DOCKING_MAX_STATE                  = 7~%int8 DOCKING_SWITCHING_TO_ML_LOC        = 7~%int8 DOCKING_MOVING_TO_APPROACH_POSE    = 6~%int8 DOCKING_SWITCHING_TO_AR_LOC        = 5~%int8 DOCKING_MOVING_TO_COMPLETE_POSE    = 4~%int8 DOCKING_CHECKING_ATTACHED          = 3~%int8 DOCKING_WAITING_FOR_SPIN_DOWN      = 2~%int8 DOCKING_SWITCHING_TO_NO_LOC        = 1~%int8 DOCKED                             = 0~%int8 UNDOCKING_SWITCHING_TO_ML_LOC      = -1~%int8 UNDOCKING_WAITING_FOR_SPIN_UP      = -2~%int8 UNDOCKING_MOVING_TO_APPROACH_POSE  = -3~%int8 UNDOCKED                           = -4~%int8 UNDOCKING_MAX_STATE                = -4~%~%# A human readble version of the (event) -> [state] transition~%string fsm_event~%string fsm_state~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DockFeedback>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DockFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'DockFeedback
    (cl:cons ':state (state msg))
))
