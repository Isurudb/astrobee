; Auto-generated. Do not edit!


(cl:in-package ff_msgs-srv)


;//! \htmlinclude SetInertia-request.msg.html

(cl:defclass <SetInertia-request> (roslisp-msg-protocol:ros-message)
  ((inertia
    :reader inertia
    :initarg :inertia
    :type geometry_msgs-msg:InertiaStamped
    :initform (cl:make-instance 'geometry_msgs-msg:InertiaStamped)))
)

(cl:defclass SetInertia-request (<SetInertia-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetInertia-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetInertia-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<SetInertia-request> is deprecated: use ff_msgs-srv:SetInertia-request instead.")))

(cl:ensure-generic-function 'inertia-val :lambda-list '(m))
(cl:defmethod inertia-val ((m <SetInertia-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:inertia-val is deprecated.  Use ff_msgs-srv:inertia instead.")
  (inertia m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetInertia-request>) ostream)
  "Serializes a message object of type '<SetInertia-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'inertia) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetInertia-request>) istream)
  "Deserializes a message object of type '<SetInertia-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'inertia) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetInertia-request>)))
  "Returns string type for a service object of type '<SetInertia-request>"
  "ff_msgs/SetInertiaRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetInertia-request)))
  "Returns string type for a service object of type 'SetInertia-request"
  "ff_msgs/SetInertiaRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetInertia-request>)))
  "Returns md5sum for a message object of type '<SetInertia-request>"
  "bed413bb59ac0b22ecff0ffcb6e7dcc2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetInertia-request)))
  "Returns md5sum for a message object of type 'SetInertia-request"
  "bed413bb59ac0b22ecff0ffcb6e7dcc2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetInertia-request>)))
  "Returns full string definition for message of type '<SetInertia-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%geometry_msgs/InertiaStamped inertia~%~%================================================================================~%MSG: geometry_msgs/InertiaStamped~%Header header~%Inertia inertia~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Inertia~%# Mass [kg]~%float64 m~%~%# Center of mass [m]~%geometry_msgs/Vector3 com~%~%# Inertia Tensor [kg-m^2]~%#     | ixx ixy ixz |~%# I = | ixy iyy iyz |~%#     | ixz iyz izz |~%float64 ixx~%float64 ixy~%float64 ixz~%float64 iyy~%float64 iyz~%float64 izz~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetInertia-request)))
  "Returns full string definition for message of type 'SetInertia-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%geometry_msgs/InertiaStamped inertia~%~%================================================================================~%MSG: geometry_msgs/InertiaStamped~%Header header~%Inertia inertia~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Inertia~%# Mass [kg]~%float64 m~%~%# Center of mass [m]~%geometry_msgs/Vector3 com~%~%# Inertia Tensor [kg-m^2]~%#     | ixx ixy ixz |~%# I = | ixy iyy iyz |~%#     | ixz iyz izz |~%float64 ixx~%float64 ixy~%float64 ixz~%float64 iyy~%float64 iyz~%float64 izz~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetInertia-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'inertia))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetInertia-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetInertia-request
    (cl:cons ':inertia (inertia msg))
))
;//! \htmlinclude SetInertia-response.msg.html

(cl:defclass <SetInertia-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetInertia-response (<SetInertia-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetInertia-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetInertia-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<SetInertia-response> is deprecated: use ff_msgs-srv:SetInertia-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetInertia-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:success-val is deprecated.  Use ff_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetInertia-response>) ostream)
  "Serializes a message object of type '<SetInertia-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetInertia-response>) istream)
  "Deserializes a message object of type '<SetInertia-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetInertia-response>)))
  "Returns string type for a service object of type '<SetInertia-response>"
  "ff_msgs/SetInertiaResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetInertia-response)))
  "Returns string type for a service object of type 'SetInertia-response"
  "ff_msgs/SetInertiaResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetInertia-response>)))
  "Returns md5sum for a message object of type '<SetInertia-response>"
  "bed413bb59ac0b22ecff0ffcb6e7dcc2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetInertia-response)))
  "Returns md5sum for a message object of type 'SetInertia-response"
  "bed413bb59ac0b22ecff0ffcb6e7dcc2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetInertia-response>)))
  "Returns full string definition for message of type '<SetInertia-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetInertia-response)))
  "Returns full string definition for message of type 'SetInertia-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetInertia-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetInertia-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetInertia-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetInertia)))
  'SetInertia-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetInertia)))
  'SetInertia-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetInertia)))
  "Returns string type for a service object of type '<SetInertia>"
  "ff_msgs/SetInertia")