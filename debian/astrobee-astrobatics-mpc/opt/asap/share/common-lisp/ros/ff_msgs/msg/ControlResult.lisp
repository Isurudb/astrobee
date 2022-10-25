; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude ControlResult.msg.html

(cl:defclass <ControlResult> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type cl:integer
    :initform 0)
   (segment
    :reader segment
    :initarg :segment
    :type (cl:vector ff_msgs-msg:ControlState)
   :initform (cl:make-array 0 :element-type 'ff_msgs-msg:ControlState :initial-element (cl:make-instance 'ff_msgs-msg:ControlState)))
   (index
    :reader index
    :initarg :index
    :type cl:integer
    :initform 0))
)

(cl:defclass ControlResult (<ControlResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<ControlResult> is deprecated: use ff_msgs-msg:ControlResult instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <ControlResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:response-val is deprecated.  Use ff_msgs-msg:response instead.")
  (response m))

(cl:ensure-generic-function 'segment-val :lambda-list '(m))
(cl:defmethod segment-val ((m <ControlResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:segment-val is deprecated.  Use ff_msgs-msg:segment instead.")
  (segment m))

(cl:ensure-generic-function 'index-val :lambda-list '(m))
(cl:defmethod index-val ((m <ControlResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:index-val is deprecated.  Use ff_msgs-msg:index instead.")
  (index m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ControlResult>)))
    "Constants for message type '<ControlResult>"
  '((:SUCCESS . 1)
    (:PREEMPTED . 0)
    (:CANCELLED . -1)
    (:CONTROL_FAILED . -2)
    (:EMPTY_SEGMENT . -3)
    (:INVALID_COMMAND . -4)
    (:TIMESYNC_ISSUE . -5))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ControlResult)))
    "Constants for message type 'ControlResult"
  '((:SUCCESS . 1)
    (:PREEMPTED . 0)
    (:CANCELLED . -1)
    (:CONTROL_FAILED . -2)
    (:EMPTY_SEGMENT . -3)
    (:INVALID_COMMAND . -4)
    (:TIMESYNC_ISSUE . -5))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlResult>) ostream)
  "Serializes a message object of type '<ControlResult>"
  (cl:let* ((signed (cl:slot-value msg 'response)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'segment))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'segment))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'index)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlResult>) istream)
  "Deserializes a message object of type '<ControlResult>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'segment) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'segment)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ff_msgs-msg:ControlState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'index)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlResult>)))
  "Returns string type for a message object of type '<ControlResult>"
  "ff_msgs/ControlResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlResult)))
  "Returns string type for a message object of type 'ControlResult"
  "ff_msgs/ControlResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlResult>)))
  "Returns md5sum for a message object of type '<ControlResult>"
  "368db35d0acc87ebfc59257b6c268764")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlResult)))
  "Returns md5sum for a message object of type 'ControlResult"
  "368db35d0acc87ebfc59257b6c268764")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlResult>)))
  "Returns full string definition for message of type '<ControlResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%int32 response                              ~%int32 SUCCESS                          =  1 # Control succeeded~%int32 PREEMPTED                        =  0 # Control preempted~%int32 CANCELLED                        = -1 # Control cancelled~%int32 CONTROL_FAILED                   = -2 # Control aborted~%int32 EMPTY_SEGMENT                    = -3 # Empty segment in goal~%int32 INVALID_COMMAND                  = -4 # Invalid command in goal~%int32 TIMESYNC_ISSUE                   = -5 # Time begins < -1s in past~%~%ff_msgs/ControlState[] segment              # Final segment flown~%uint32 index                                # Index being processed~%~%~%================================================================================~%MSG: ff_msgs/ControlState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Full state vector containing Time, Pose, Vel, and Accel~%# ~%# when {time}~%# flight_mode {string} - disctates, gains, tolerances, etc.~%# pose {Point position, Quaternion orientation}~%# twist {Vector3 linear, Vector3 angular}~%# accel {Vector3 linear, Vector3 angular}~%~%time when~%geometry_msgs/Pose pose~%geometry_msgs/Twist twist~%geometry_msgs/Twist accel~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlResult)))
  "Returns full string definition for message of type 'ControlResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%int32 response                              ~%int32 SUCCESS                          =  1 # Control succeeded~%int32 PREEMPTED                        =  0 # Control preempted~%int32 CANCELLED                        = -1 # Control cancelled~%int32 CONTROL_FAILED                   = -2 # Control aborted~%int32 EMPTY_SEGMENT                    = -3 # Empty segment in goal~%int32 INVALID_COMMAND                  = -4 # Invalid command in goal~%int32 TIMESYNC_ISSUE                   = -5 # Time begins < -1s in past~%~%ff_msgs/ControlState[] segment              # Final segment flown~%uint32 index                                # Index being processed~%~%~%================================================================================~%MSG: ff_msgs/ControlState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Full state vector containing Time, Pose, Vel, and Accel~%# ~%# when {time}~%# flight_mode {string} - disctates, gains, tolerances, etc.~%# pose {Point position, Quaternion orientation}~%# twist {Vector3 linear, Vector3 angular}~%# accel {Vector3 linear, Vector3 angular}~%~%time when~%geometry_msgs/Pose pose~%geometry_msgs/Twist twist~%geometry_msgs/Twist accel~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlResult>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'segment) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlResult>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlResult
    (cl:cons ':response (response msg))
    (cl:cons ':segment (segment msg))
    (cl:cons ':index (index msg))
))
