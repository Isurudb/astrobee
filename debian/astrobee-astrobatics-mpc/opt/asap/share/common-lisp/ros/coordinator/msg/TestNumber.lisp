; Auto-generated. Do not edit!


(cl:in-package coordinator-msg)


;//! \htmlinclude TestNumber.msg.html

(cl:defclass <TestNumber> (roslisp-msg-protocol:ros-message)
  ((stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0)
   (test_number
    :reader test_number
    :initarg :test_number
    :type cl:integer
    :initform 0)
   (role
    :reader role
    :initarg :role
    :type cl:string
    :initform ""))
)

(cl:defclass TestNumber (<TestNumber>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TestNumber>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TestNumber)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name coordinator-msg:<TestNumber> is deprecated: use coordinator-msg:TestNumber instead.")))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <TestNumber>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coordinator-msg:stamp-val is deprecated.  Use coordinator-msg:stamp instead.")
  (stamp m))

(cl:ensure-generic-function 'test_number-val :lambda-list '(m))
(cl:defmethod test_number-val ((m <TestNumber>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coordinator-msg:test_number-val is deprecated.  Use coordinator-msg:test_number instead.")
  (test_number m))

(cl:ensure-generic-function 'role-val :lambda-list '(m))
(cl:defmethod role-val ((m <TestNumber>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coordinator-msg:role-val is deprecated.  Use coordinator-msg:role instead.")
  (role m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TestNumber>) ostream)
  "Serializes a message object of type '<TestNumber>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'stamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'stamp) (cl:floor (cl:slot-value msg 'stamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let* ((signed (cl:slot-value msg 'test_number)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'role))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'role))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TestNumber>) istream)
  "Deserializes a message object of type '<TestNumber>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'test_number) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'role) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'role) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TestNumber>)))
  "Returns string type for a message object of type '<TestNumber>"
  "coordinator/TestNumber")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TestNumber)))
  "Returns string type for a message object of type 'TestNumber"
  "coordinator/TestNumber")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TestNumber>)))
  "Returns md5sum for a message object of type '<TestNumber>"
  "a546cf58ee360e93604091100205de8f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TestNumber)))
  "Returns md5sum for a message object of type 'TestNumber"
  "a546cf58ee360e93604091100205de8f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TestNumber>)))
  "Returns full string definition for message of type '<TestNumber>"
  (cl:format cl:nil "time stamp~%int32 test_number~%string role~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TestNumber)))
  "Returns full string definition for message of type 'TestNumber"
  (cl:format cl:nil "time stamp~%int32 test_number~%string role~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TestNumber>))
  (cl:+ 0
     8
     4
     4 (cl:length (cl:slot-value msg 'role))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TestNumber>))
  "Converts a ROS message object to a list"
  (cl:list 'TestNumber
    (cl:cons ':stamp (stamp msg))
    (cl:cons ':test_number (test_number msg))
    (cl:cons ':role (role msg))
))
