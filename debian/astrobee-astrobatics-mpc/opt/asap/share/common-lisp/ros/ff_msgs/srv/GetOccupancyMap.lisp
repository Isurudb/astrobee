; Auto-generated. Do not edit!


(cl:in-package ff_msgs-srv)


;//! \htmlinclude GetOccupancyMap-request.msg.html

(cl:defclass <GetOccupancyMap-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetOccupancyMap-request (<GetOccupancyMap-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetOccupancyMap-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetOccupancyMap-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<GetOccupancyMap-request> is deprecated: use ff_msgs-srv:GetOccupancyMap-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetOccupancyMap-request>) ostream)
  "Serializes a message object of type '<GetOccupancyMap-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetOccupancyMap-request>) istream)
  "Deserializes a message object of type '<GetOccupancyMap-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetOccupancyMap-request>)))
  "Returns string type for a service object of type '<GetOccupancyMap-request>"
  "ff_msgs/GetOccupancyMapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetOccupancyMap-request)))
  "Returns string type for a service object of type 'GetOccupancyMap-request"
  "ff_msgs/GetOccupancyMapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetOccupancyMap-request>)))
  "Returns md5sum for a message object of type '<GetOccupancyMap-request>"
  "7424633971d7f4a5e61061b696c2c185")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetOccupancyMap-request)))
  "Returns md5sum for a message object of type 'GetOccupancyMap-request"
  "7424633971d7f4a5e61061b696c2c185")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetOccupancyMap-request>)))
  "Returns full string definition for message of type '<GetOccupancyMap-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetOccupancyMap-request)))
  "Returns full string definition for message of type 'GetOccupancyMap-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetOccupancyMap-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetOccupancyMap-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetOccupancyMap-request
))
;//! \htmlinclude GetOccupancyMap-response.msg.html

(cl:defclass <GetOccupancyMap-response> (roslisp-msg-protocol:ros-message)
  ((timestamp
    :reader timestamp
    :initarg :timestamp
    :type cl:real
    :initform 0)
   (map
    :reader map
    :initarg :map
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (origin
    :reader origin
    :initarg :origin
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (dim
    :reader dim
    :initarg :dim
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (resolution
    :reader resolution
    :initarg :resolution
    :type cl:float
    :initform 0.0))
)

(cl:defclass GetOccupancyMap-response (<GetOccupancyMap-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetOccupancyMap-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetOccupancyMap-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<GetOccupancyMap-response> is deprecated: use ff_msgs-srv:GetOccupancyMap-response instead.")))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <GetOccupancyMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:timestamp-val is deprecated.  Use ff_msgs-srv:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'map-val :lambda-list '(m))
(cl:defmethod map-val ((m <GetOccupancyMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:map-val is deprecated.  Use ff_msgs-srv:map instead.")
  (map m))

(cl:ensure-generic-function 'origin-val :lambda-list '(m))
(cl:defmethod origin-val ((m <GetOccupancyMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:origin-val is deprecated.  Use ff_msgs-srv:origin instead.")
  (origin m))

(cl:ensure-generic-function 'dim-val :lambda-list '(m))
(cl:defmethod dim-val ((m <GetOccupancyMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:dim-val is deprecated.  Use ff_msgs-srv:dim instead.")
  (dim m))

(cl:ensure-generic-function 'resolution-val :lambda-list '(m))
(cl:defmethod resolution-val ((m <GetOccupancyMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:resolution-val is deprecated.  Use ff_msgs-srv:resolution instead.")
  (resolution m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetOccupancyMap-response>) ostream)
  "Serializes a message object of type '<GetOccupancyMap-response>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'timestamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'timestamp) (cl:floor (cl:slot-value msg 'timestamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'map))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'map))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'origin) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'dim) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'resolution))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetOccupancyMap-response>) istream)
  "Deserializes a message object of type '<GetOccupancyMap-response>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'timestamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'map) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'map)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'origin) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'dim) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'resolution) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetOccupancyMap-response>)))
  "Returns string type for a service object of type '<GetOccupancyMap-response>"
  "ff_msgs/GetOccupancyMapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetOccupancyMap-response)))
  "Returns string type for a service object of type 'GetOccupancyMap-response"
  "ff_msgs/GetOccupancyMapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetOccupancyMap-response>)))
  "Returns md5sum for a message object of type '<GetOccupancyMap-response>"
  "7424633971d7f4a5e61061b696c2c185")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetOccupancyMap-response)))
  "Returns md5sum for a message object of type 'GetOccupancyMap-response"
  "7424633971d7f4a5e61061b696c2c185")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetOccupancyMap-response>)))
  "Returns full string definition for message of type '<GetOccupancyMap-response>"
  (cl:format cl:nil "time timestamp~%~%int8[] map~%geometry_msgs/Vector3 origin~%geometry_msgs/Vector3 dim~%float32 resolution~%~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetOccupancyMap-response)))
  "Returns full string definition for message of type 'GetOccupancyMap-response"
  (cl:format cl:nil "time timestamp~%~%int8[] map~%geometry_msgs/Vector3 origin~%geometry_msgs/Vector3 dim~%float32 resolution~%~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetOccupancyMap-response>))
  (cl:+ 0
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'map) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'origin))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'dim))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetOccupancyMap-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetOccupancyMap-response
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':map (map msg))
    (cl:cons ':origin (origin msg))
    (cl:cons ':dim (dim msg))
    (cl:cons ':resolution (resolution msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetOccupancyMap)))
  'GetOccupancyMap-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetOccupancyMap)))
  'GetOccupancyMap-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetOccupancyMap)))
  "Returns string type for a service object of type '<GetOccupancyMap>"
  "ff_msgs/GetOccupancyMap")