; Auto-generated. Do not edit!


(cl:in-package coordinator-msg)


;//! \htmlinclude StatusPrimary.msg.html

(cl:defclass <StatusPrimary> (roslisp-msg-protocol:ros-message)
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
   (default_control
    :reader default_control
    :initarg :default_control
    :type cl:boolean
    :initform cl:nil)
   (flight_mode
    :reader flight_mode
    :initarg :flight_mode
    :type cl:string
    :initform "")
   (test_finished
    :reader test_finished
    :initarg :test_finished
    :type cl:boolean
    :initform cl:nil)
   (coord_ok
    :reader coord_ok
    :initarg :coord_ok
    :type cl:boolean
    :initform cl:nil)
   (control_mode
    :reader control_mode
    :initarg :control_mode
    :type cl:string
    :initform "")
   (regulate_finished
    :reader regulate_finished
    :initarg :regulate_finished
    :type cl:boolean
    :initform cl:nil)
   (description
    :reader description
    :initarg :description
    :type cl:string
    :initform ""))
)

(cl:defclass StatusPrimary (<StatusPrimary>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StatusPrimary>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StatusPrimary)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name coordinator-msg:<StatusPrimary> is deprecated: use coordinator-msg:StatusPrimary instead.")))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <StatusPrimary>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coordinator-msg:stamp-val is deprecated.  Use coordinator-msg:stamp instead.")
  (stamp m))

(cl:ensure-generic-function 'test_number-val :lambda-list '(m))
(cl:defmethod test_number-val ((m <StatusPrimary>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coordinator-msg:test_number-val is deprecated.  Use coordinator-msg:test_number instead.")
  (test_number m))

(cl:ensure-generic-function 'default_control-val :lambda-list '(m))
(cl:defmethod default_control-val ((m <StatusPrimary>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coordinator-msg:default_control-val is deprecated.  Use coordinator-msg:default_control instead.")
  (default_control m))

(cl:ensure-generic-function 'flight_mode-val :lambda-list '(m))
(cl:defmethod flight_mode-val ((m <StatusPrimary>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coordinator-msg:flight_mode-val is deprecated.  Use coordinator-msg:flight_mode instead.")
  (flight_mode m))

(cl:ensure-generic-function 'test_finished-val :lambda-list '(m))
(cl:defmethod test_finished-val ((m <StatusPrimary>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coordinator-msg:test_finished-val is deprecated.  Use coordinator-msg:test_finished instead.")
  (test_finished m))

(cl:ensure-generic-function 'coord_ok-val :lambda-list '(m))
(cl:defmethod coord_ok-val ((m <StatusPrimary>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coordinator-msg:coord_ok-val is deprecated.  Use coordinator-msg:coord_ok instead.")
  (coord_ok m))

(cl:ensure-generic-function 'control_mode-val :lambda-list '(m))
(cl:defmethod control_mode-val ((m <StatusPrimary>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coordinator-msg:control_mode-val is deprecated.  Use coordinator-msg:control_mode instead.")
  (control_mode m))

(cl:ensure-generic-function 'regulate_finished-val :lambda-list '(m))
(cl:defmethod regulate_finished-val ((m <StatusPrimary>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coordinator-msg:regulate_finished-val is deprecated.  Use coordinator-msg:regulate_finished instead.")
  (regulate_finished m))

(cl:ensure-generic-function 'description-val :lambda-list '(m))
(cl:defmethod description-val ((m <StatusPrimary>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coordinator-msg:description-val is deprecated.  Use coordinator-msg:description instead.")
  (description m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StatusPrimary>) ostream)
  "Serializes a message object of type '<StatusPrimary>"
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'default_control) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'flight_mode))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'flight_mode))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'test_finished) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'coord_ok) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'control_mode))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'control_mode))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'regulate_finished) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'description))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'description))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StatusPrimary>) istream)
  "Deserializes a message object of type '<StatusPrimary>"
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
    (cl:setf (cl:slot-value msg 'default_control) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'flight_mode) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'flight_mode) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'test_finished) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'coord_ok) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'control_mode) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'control_mode) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'regulate_finished) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'description) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'description) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StatusPrimary>)))
  "Returns string type for a message object of type '<StatusPrimary>"
  "coordinator/StatusPrimary")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StatusPrimary)))
  "Returns string type for a message object of type 'StatusPrimary"
  "coordinator/StatusPrimary")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StatusPrimary>)))
  "Returns md5sum for a message object of type '<StatusPrimary>"
  "7b1f7333b81ba92ccdc2c10a9400c075")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StatusPrimary)))
  "Returns md5sum for a message object of type 'StatusPrimary"
  "7b1f7333b81ba92ccdc2c10a9400c075")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StatusPrimary>)))
  "Returns full string definition for message of type '<StatusPrimary>"
  (cl:format cl:nil "time stamp~%~%# base (shared) values~%int32 test_number~%bool default_control~%string flight_mode~%bool test_finished~%bool coord_ok~%~%# StatusPrimary~%string control_mode  # did something break?~%bool regulate_finished~%string description  # for telemetry description~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StatusPrimary)))
  "Returns full string definition for message of type 'StatusPrimary"
  (cl:format cl:nil "time stamp~%~%# base (shared) values~%int32 test_number~%bool default_control~%string flight_mode~%bool test_finished~%bool coord_ok~%~%# StatusPrimary~%string control_mode  # did something break?~%bool regulate_finished~%string description  # for telemetry description~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StatusPrimary>))
  (cl:+ 0
     8
     4
     1
     4 (cl:length (cl:slot-value msg 'flight_mode))
     1
     1
     4 (cl:length (cl:slot-value msg 'control_mode))
     1
     4 (cl:length (cl:slot-value msg 'description))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StatusPrimary>))
  "Converts a ROS message object to a list"
  (cl:list 'StatusPrimary
    (cl:cons ':stamp (stamp msg))
    (cl:cons ':test_number (test_number msg))
    (cl:cons ':default_control (default_control msg))
    (cl:cons ':flight_mode (flight_mode msg))
    (cl:cons ':test_finished (test_finished msg))
    (cl:cons ':coord_ok (coord_ok msg))
    (cl:cons ':control_mode (control_mode msg))
    (cl:cons ':regulate_finished (regulate_finished msg))
    (cl:cons ':description (description msg))
))
