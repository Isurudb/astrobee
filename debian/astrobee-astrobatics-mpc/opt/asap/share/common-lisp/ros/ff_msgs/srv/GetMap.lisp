; Auto-generated. Do not edit!


(cl:in-package ff_msgs-srv)


;//! \htmlinclude GetMap-request.msg.html

(cl:defclass <GetMap-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetMap-request (<GetMap-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetMap-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetMap-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<GetMap-request> is deprecated: use ff_msgs-srv:GetMap-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetMap-request>) ostream)
  "Serializes a message object of type '<GetMap-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetMap-request>) istream)
  "Deserializes a message object of type '<GetMap-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetMap-request>)))
  "Returns string type for a service object of type '<GetMap-request>"
  "ff_msgs/GetMapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMap-request)))
  "Returns string type for a service object of type 'GetMap-request"
  "ff_msgs/GetMapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetMap-request>)))
  "Returns md5sum for a message object of type '<GetMap-request>"
  "2fabf4f6566443e23e62f744b9ba2ad6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetMap-request)))
  "Returns md5sum for a message object of type 'GetMap-request"
  "2fabf4f6566443e23e62f744b9ba2ad6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetMap-request>)))
  "Returns full string definition for message of type '<GetMap-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetMap-request)))
  "Returns full string definition for message of type 'GetMap-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetMap-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetMap-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetMap-request
))
;//! \htmlinclude GetMap-response.msg.html

(cl:defclass <GetMap-response> (roslisp-msg-protocol:ros-message)
  ((points
    :reader points
    :initarg :points
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (resolution
    :reader resolution
    :initarg :resolution
    :type cl:float
    :initform 0.0)
   (free
    :reader free
    :initarg :free
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetMap-response (<GetMap-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetMap-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetMap-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<GetMap-response> is deprecated: use ff_msgs-srv:GetMap-response instead.")))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <GetMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:points-val is deprecated.  Use ff_msgs-srv:points instead.")
  (points m))

(cl:ensure-generic-function 'resolution-val :lambda-list '(m))
(cl:defmethod resolution-val ((m <GetMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:resolution-val is deprecated.  Use ff_msgs-srv:resolution instead.")
  (resolution m))

(cl:ensure-generic-function 'free-val :lambda-list '(m))
(cl:defmethod free-val ((m <GetMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:free-val is deprecated.  Use ff_msgs-srv:free instead.")
  (free m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetMap-response>) ostream)
  "Serializes a message object of type '<GetMap-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'points) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'resolution))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'free) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetMap-response>) istream)
  "Deserializes a message object of type '<GetMap-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'points) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'resolution) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'free) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetMap-response>)))
  "Returns string type for a service object of type '<GetMap-response>"
  "ff_msgs/GetMapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMap-response)))
  "Returns string type for a service object of type 'GetMap-response"
  "ff_msgs/GetMapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetMap-response>)))
  "Returns md5sum for a message object of type '<GetMap-response>"
  "2fabf4f6566443e23e62f744b9ba2ad6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetMap-response)))
  "Returns md5sum for a message object of type 'GetMap-response"
  "2fabf4f6566443e23e62f744b9ba2ad6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetMap-response>)))
  "Returns full string definition for message of type '<GetMap-response>"
  (cl:format cl:nil "sensor_msgs/PointCloud2 points~%float32 resolution~%bool free~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetMap-response)))
  "Returns full string definition for message of type 'GetMap-response"
  (cl:format cl:nil "sensor_msgs/PointCloud2 points~%float32 resolution~%bool free~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetMap-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'points))
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetMap-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetMap-response
    (cl:cons ':points (points msg))
    (cl:cons ':resolution (resolution msg))
    (cl:cons ':free (free msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetMap)))
  'GetMap-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetMap)))
  'GetMap-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMap)))
  "Returns string type for a service object of type '<GetMap>"
  "ff_msgs/GetMap")