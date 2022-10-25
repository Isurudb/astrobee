; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude ArmFeedback.msg.html

(cl:defclass <ArmFeedback> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type ff_msgs-msg:ArmState
    :initform (cl:make-instance 'ff_msgs-msg:ArmState))
   (pan
    :reader pan
    :initarg :pan
    :type cl:float
    :initform 0.0)
   (tilt
    :reader tilt
    :initarg :tilt
    :type cl:float
    :initform 0.0)
   (gripper
    :reader gripper
    :initarg :gripper
    :type cl:float
    :initform 0.0))
)

(cl:defclass ArmFeedback (<ArmFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<ArmFeedback> is deprecated: use ff_msgs-msg:ArmFeedback instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <ArmFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:state-val is deprecated.  Use ff_msgs-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'pan-val :lambda-list '(m))
(cl:defmethod pan-val ((m <ArmFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:pan-val is deprecated.  Use ff_msgs-msg:pan instead.")
  (pan m))

(cl:ensure-generic-function 'tilt-val :lambda-list '(m))
(cl:defmethod tilt-val ((m <ArmFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:tilt-val is deprecated.  Use ff_msgs-msg:tilt instead.")
  (tilt m))

(cl:ensure-generic-function 'gripper-val :lambda-list '(m))
(cl:defmethod gripper-val ((m <ArmFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:gripper-val is deprecated.  Use ff_msgs-msg:gripper instead.")
  (gripper m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmFeedback>) ostream)
  "Serializes a message object of type '<ArmFeedback>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'state) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pan))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tilt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'gripper))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmFeedback>) istream)
  "Deserializes a message object of type '<ArmFeedback>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'state) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pan) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tilt) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gripper) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmFeedback>)))
  "Returns string type for a message object of type '<ArmFeedback>"
  "ff_msgs/ArmFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmFeedback)))
  "Returns string type for a message object of type 'ArmFeedback"
  "ff_msgs/ArmFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmFeedback>)))
  "Returns md5sum for a message object of type '<ArmFeedback>"
  "2071124ffef61efc1590237720618232")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmFeedback)))
  "Returns md5sum for a message object of type 'ArmFeedback"
  "2071124ffef61efc1590237720618232")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmFeedback>)))
  "Returns full string definition for message of type '<ArmFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%ff_msgs/ArmState state                        # Complete state~%~%float32 pan                                   # Current PAN value~%float32 tilt                                  # Current TILT value~%float32 gripper                               # Current GRIPPER value~%~%~%================================================================================~%MSG: ff_msgs/ArmState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# The state of the arm behavior~%~%# Header with timestamp~%std_msgs/Header header~%~%# Tee current state~%int8 state                         # Current state~%int8 INITIALIZING        = 0       # Waiting on child services, actions, etc.~%int8 UNKNOWN             = 1       # Waiting on feedback from driver~%int8 STOWED              = 2       # The arm is stowed~%int8 DEPLOYED            = 3       # The arm is deployed~%int8 SETTING             = 4       # The gripper is being set to a value~%int8 PANNING             = 5       # We are panning as part of a move~%int8 TILTING             = 6       # We are tilting as part of a move~%int8 STOWING_SETTING     = 7       # We are closing the gripper for stowing~%int8 STOWING_PANNING     = 8       # We are panning to zero for stowing~%int8 STOWING_TILTING     = 9       # We are tilting to zero for stowing~%int8 DEPLOYING_PANNING   = 10      # We are panning to zero for stowing~%int8 DEPLOYING_TILTING   = 11      # We are tilting to zero for stowing~%int8 CALIBRATING         = 12      # We are calibrating the gripper~%~%# A human readble version of the (event) -> [state] transition~%string fsm_event~%string fsm_state~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmFeedback)))
  "Returns full string definition for message of type 'ArmFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%ff_msgs/ArmState state                        # Complete state~%~%float32 pan                                   # Current PAN value~%float32 tilt                                  # Current TILT value~%float32 gripper                               # Current GRIPPER value~%~%~%================================================================================~%MSG: ff_msgs/ArmState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# The state of the arm behavior~%~%# Header with timestamp~%std_msgs/Header header~%~%# Tee current state~%int8 state                         # Current state~%int8 INITIALIZING        = 0       # Waiting on child services, actions, etc.~%int8 UNKNOWN             = 1       # Waiting on feedback from driver~%int8 STOWED              = 2       # The arm is stowed~%int8 DEPLOYED            = 3       # The arm is deployed~%int8 SETTING             = 4       # The gripper is being set to a value~%int8 PANNING             = 5       # We are panning as part of a move~%int8 TILTING             = 6       # We are tilting as part of a move~%int8 STOWING_SETTING     = 7       # We are closing the gripper for stowing~%int8 STOWING_PANNING     = 8       # We are panning to zero for stowing~%int8 STOWING_TILTING     = 9       # We are tilting to zero for stowing~%int8 DEPLOYING_PANNING   = 10      # We are panning to zero for stowing~%int8 DEPLOYING_TILTING   = 11      # We are tilting to zero for stowing~%int8 CALIBRATING         = 12      # We are calibrating the gripper~%~%# A human readble version of the (event) -> [state] transition~%string fsm_event~%string fsm_state~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmFeedback>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'state))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmFeedback
    (cl:cons ':state (state msg))
    (cl:cons ':pan (pan msg))
    (cl:cons ':tilt (tilt msg))
    (cl:cons ':gripper (gripper msg))
))
