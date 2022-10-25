; Auto-generated. Do not edit!


(cl:in-package ff_msgs-srv)


;//! \htmlinclude GetPipelines-request.msg.html

(cl:defclass <GetPipelines-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetPipelines-request (<GetPipelines-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPipelines-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPipelines-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<GetPipelines-request> is deprecated: use ff_msgs-srv:GetPipelines-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPipelines-request>) ostream)
  "Serializes a message object of type '<GetPipelines-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPipelines-request>) istream)
  "Deserializes a message object of type '<GetPipelines-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPipelines-request>)))
  "Returns string type for a service object of type '<GetPipelines-request>"
  "ff_msgs/GetPipelinesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPipelines-request)))
  "Returns string type for a service object of type 'GetPipelines-request"
  "ff_msgs/GetPipelinesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPipelines-request>)))
  "Returns md5sum for a message object of type '<GetPipelines-request>"
  "4fb31d141d0f152e9301905ffcaa8f48")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPipelines-request)))
  "Returns md5sum for a message object of type 'GetPipelines-request"
  "4fb31d141d0f152e9301905ffcaa8f48")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPipelines-request>)))
  "Returns full string definition for message of type '<GetPipelines-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPipelines-request)))
  "Returns full string definition for message of type 'GetPipelines-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPipelines-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPipelines-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPipelines-request
))
;//! \htmlinclude GetPipelines-response.msg.html

(cl:defclass <GetPipelines-response> (roslisp-msg-protocol:ros-message)
  ((pipelines
    :reader pipelines
    :initarg :pipelines
    :type (cl:vector ff_msgs-msg:LocalizationPipeline)
   :initform (cl:make-array 0 :element-type 'ff_msgs-msg:LocalizationPipeline :initial-element (cl:make-instance 'ff_msgs-msg:LocalizationPipeline))))
)

(cl:defclass GetPipelines-response (<GetPipelines-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPipelines-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPipelines-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<GetPipelines-response> is deprecated: use ff_msgs-srv:GetPipelines-response instead.")))

(cl:ensure-generic-function 'pipelines-val :lambda-list '(m))
(cl:defmethod pipelines-val ((m <GetPipelines-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:pipelines-val is deprecated.  Use ff_msgs-srv:pipelines instead.")
  (pipelines m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPipelines-response>) ostream)
  "Serializes a message object of type '<GetPipelines-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pipelines))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'pipelines))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPipelines-response>) istream)
  "Deserializes a message object of type '<GetPipelines-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pipelines) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pipelines)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ff_msgs-msg:LocalizationPipeline))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPipelines-response>)))
  "Returns string type for a service object of type '<GetPipelines-response>"
  "ff_msgs/GetPipelinesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPipelines-response)))
  "Returns string type for a service object of type 'GetPipelines-response"
  "ff_msgs/GetPipelinesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPipelines-response>)))
  "Returns md5sum for a message object of type '<GetPipelines-response>"
  "4fb31d141d0f152e9301905ffcaa8f48")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPipelines-response)))
  "Returns md5sum for a message object of type 'GetPipelines-response"
  "4fb31d141d0f152e9301905ffcaa8f48")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPipelines-response>)))
  "Returns full string definition for message of type '<GetPipelines-response>"
  (cl:format cl:nil "~%ff_msgs/LocalizationPipeline[] pipelines~%~%~%================================================================================~%MSG: ff_msgs/LocalizationPipeline~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Information about a pipeline~%~%string id                     # Short id for the pipeline~%uint8 mode                    # EKF mode for the pipeline~%string name                   # Long name for the pipe~%bool requires_filter          # Does this pipeline require the EKF~%bool requires_optical_flow    # Does this pipeline require optical flow~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPipelines-response)))
  "Returns full string definition for message of type 'GetPipelines-response"
  (cl:format cl:nil "~%ff_msgs/LocalizationPipeline[] pipelines~%~%~%================================================================================~%MSG: ff_msgs/LocalizationPipeline~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Information about a pipeline~%~%string id                     # Short id for the pipeline~%uint8 mode                    # EKF mode for the pipeline~%string name                   # Long name for the pipe~%bool requires_filter          # Does this pipeline require the EKF~%bool requires_optical_flow    # Does this pipeline require optical flow~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPipelines-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pipelines) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPipelines-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPipelines-response
    (cl:cons ':pipelines (pipelines msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetPipelines)))
  'GetPipelines-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetPipelines)))
  'GetPipelines-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPipelines)))
  "Returns string type for a service object of type '<GetPipelines>"
  "ff_msgs/GetPipelines")