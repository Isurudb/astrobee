; Auto-generated. Do not edit!


(cl:in-package ff_msgs-srv)


;//! \htmlinclude UnloadLoadNodelet-request.msg.html

(cl:defclass <UnloadLoadNodelet-request> (roslisp-msg-protocol:ros-message)
  ((load
    :reader load
    :initarg :load
    :type cl:boolean
    :initform cl:nil)
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (type
    :reader type
    :initarg :type
    :type cl:string
    :initform "")
   (manager_name
    :reader manager_name
    :initarg :manager_name
    :type cl:string
    :initform "")
   (remap_source_args
    :reader remap_source_args
    :initarg :remap_source_args
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (remap_target_args
    :reader remap_target_args
    :initarg :remap_target_args
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (my_argv
    :reader my_argv
    :initarg :my_argv
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (bond_id
    :reader bond_id
    :initarg :bond_id
    :type cl:string
    :initform ""))
)

(cl:defclass UnloadLoadNodelet-request (<UnloadLoadNodelet-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UnloadLoadNodelet-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UnloadLoadNodelet-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<UnloadLoadNodelet-request> is deprecated: use ff_msgs-srv:UnloadLoadNodelet-request instead.")))

(cl:ensure-generic-function 'load-val :lambda-list '(m))
(cl:defmethod load-val ((m <UnloadLoadNodelet-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:load-val is deprecated.  Use ff_msgs-srv:load instead.")
  (load m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <UnloadLoadNodelet-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:name-val is deprecated.  Use ff_msgs-srv:name instead.")
  (name m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <UnloadLoadNodelet-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:type-val is deprecated.  Use ff_msgs-srv:type instead.")
  (type m))

(cl:ensure-generic-function 'manager_name-val :lambda-list '(m))
(cl:defmethod manager_name-val ((m <UnloadLoadNodelet-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:manager_name-val is deprecated.  Use ff_msgs-srv:manager_name instead.")
  (manager_name m))

(cl:ensure-generic-function 'remap_source_args-val :lambda-list '(m))
(cl:defmethod remap_source_args-val ((m <UnloadLoadNodelet-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:remap_source_args-val is deprecated.  Use ff_msgs-srv:remap_source_args instead.")
  (remap_source_args m))

(cl:ensure-generic-function 'remap_target_args-val :lambda-list '(m))
(cl:defmethod remap_target_args-val ((m <UnloadLoadNodelet-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:remap_target_args-val is deprecated.  Use ff_msgs-srv:remap_target_args instead.")
  (remap_target_args m))

(cl:ensure-generic-function 'my_argv-val :lambda-list '(m))
(cl:defmethod my_argv-val ((m <UnloadLoadNodelet-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:my_argv-val is deprecated.  Use ff_msgs-srv:my_argv instead.")
  (my_argv m))

(cl:ensure-generic-function 'bond_id-val :lambda-list '(m))
(cl:defmethod bond_id-val ((m <UnloadLoadNodelet-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:bond_id-val is deprecated.  Use ff_msgs-srv:bond_id instead.")
  (bond_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UnloadLoadNodelet-request>) ostream)
  "Serializes a message object of type '<UnloadLoadNodelet-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'load) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'type))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'manager_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'manager_name))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'remap_source_args))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'remap_source_args))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'remap_target_args))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'remap_target_args))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'my_argv))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'my_argv))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'bond_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'bond_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UnloadLoadNodelet-request>) istream)
  "Deserializes a message object of type '<UnloadLoadNodelet-request>"
    (cl:setf (cl:slot-value msg 'load) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'manager_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'manager_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'remap_source_args) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'remap_source_args)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'remap_target_args) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'remap_target_args)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'my_argv) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'my_argv)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'bond_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'bond_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UnloadLoadNodelet-request>)))
  "Returns string type for a service object of type '<UnloadLoadNodelet-request>"
  "ff_msgs/UnloadLoadNodeletRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UnloadLoadNodelet-request)))
  "Returns string type for a service object of type 'UnloadLoadNodelet-request"
  "ff_msgs/UnloadLoadNodeletRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UnloadLoadNodelet-request>)))
  "Returns md5sum for a message object of type '<UnloadLoadNodelet-request>"
  "7f19eb1a2a34b5a95695a9d88b20e227")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UnloadLoadNodelet-request)))
  "Returns md5sum for a message object of type 'UnloadLoadNodelet-request"
  "7f19eb1a2a34b5a95695a9d88b20e227")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UnloadLoadNodelet-request>)))
  "Returns full string definition for message of type '<UnloadLoadNodelet-request>"
  (cl:format cl:nil "~%~%~%~%~%~%bool load~%~%string name~%~%~%~%string type~%~%~%~%string manager_name~%~%string[] remap_source_args~%string[] remap_target_args~%string[] my_argv~%string bond_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UnloadLoadNodelet-request)))
  "Returns full string definition for message of type 'UnloadLoadNodelet-request"
  (cl:format cl:nil "~%~%~%~%~%~%bool load~%~%string name~%~%~%~%string type~%~%~%~%string manager_name~%~%string[] remap_source_args~%string[] remap_target_args~%string[] my_argv~%string bond_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UnloadLoadNodelet-request>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:length (cl:slot-value msg 'type))
     4 (cl:length (cl:slot-value msg 'manager_name))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'remap_source_args) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'remap_target_args) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'my_argv) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:length (cl:slot-value msg 'bond_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UnloadLoadNodelet-request>))
  "Converts a ROS message object to a list"
  (cl:list 'UnloadLoadNodelet-request
    (cl:cons ':load (load msg))
    (cl:cons ':name (name msg))
    (cl:cons ':type (type msg))
    (cl:cons ':manager_name (manager_name msg))
    (cl:cons ':remap_source_args (remap_source_args msg))
    (cl:cons ':remap_target_args (remap_target_args msg))
    (cl:cons ':my_argv (my_argv msg))
    (cl:cons ':bond_id (bond_id msg))
))
;//! \htmlinclude UnloadLoadNodelet-response.msg.html

(cl:defclass <UnloadLoadNodelet-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0))
)

(cl:defclass UnloadLoadNodelet-response (<UnloadLoadNodelet-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UnloadLoadNodelet-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UnloadLoadNodelet-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<UnloadLoadNodelet-response> is deprecated: use ff_msgs-srv:UnloadLoadNodelet-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <UnloadLoadNodelet-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:result-val is deprecated.  Use ff_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<UnloadLoadNodelet-response>)))
    "Constants for message type '<UnloadLoadNodelet-response>"
  '((:SUCCESSFUL . 1)
    (:ROS_SERVICE_FAILED . 2)
    (:NODE_NOT_IN_MAP . 3)
    (:MANAGER_NAME_MISSING . 4)
    (:TYPE_MISSING . 5))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'UnloadLoadNodelet-response)))
    "Constants for message type 'UnloadLoadNodelet-response"
  '((:SUCCESSFUL . 1)
    (:ROS_SERVICE_FAILED . 2)
    (:NODE_NOT_IN_MAP . 3)
    (:MANAGER_NAME_MISSING . 4)
    (:TYPE_MISSING . 5))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UnloadLoadNodelet-response>) ostream)
  "Serializes a message object of type '<UnloadLoadNodelet-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UnloadLoadNodelet-response>) istream)
  "Deserializes a message object of type '<UnloadLoadNodelet-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UnloadLoadNodelet-response>)))
  "Returns string type for a service object of type '<UnloadLoadNodelet-response>"
  "ff_msgs/UnloadLoadNodeletResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UnloadLoadNodelet-response)))
  "Returns string type for a service object of type 'UnloadLoadNodelet-response"
  "ff_msgs/UnloadLoadNodeletResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UnloadLoadNodelet-response>)))
  "Returns md5sum for a message object of type '<UnloadLoadNodelet-response>"
  "7f19eb1a2a34b5a95695a9d88b20e227")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UnloadLoadNodelet-response)))
  "Returns md5sum for a message object of type 'UnloadLoadNodelet-response"
  "7f19eb1a2a34b5a95695a9d88b20e227")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UnloadLoadNodelet-response>)))
  "Returns full string definition for message of type '<UnloadLoadNodelet-response>"
  (cl:format cl:nil "int32 result~%~%int32 SUCCESSFUL            = 1~%~%int32 ROS_SERVICE_FAILED    = 2~%~%~%~%int32 NODE_NOT_IN_MAP       = 3~%~%~%int32 MANAGER_NAME_MISSING  = 4~%~%~%int32 TYPE_MISSING          = 5~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UnloadLoadNodelet-response)))
  "Returns full string definition for message of type 'UnloadLoadNodelet-response"
  (cl:format cl:nil "int32 result~%~%int32 SUCCESSFUL            = 1~%~%int32 ROS_SERVICE_FAILED    = 2~%~%~%~%int32 NODE_NOT_IN_MAP       = 3~%~%~%int32 MANAGER_NAME_MISSING  = 4~%~%~%int32 TYPE_MISSING          = 5~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UnloadLoadNodelet-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UnloadLoadNodelet-response>))
  "Converts a ROS message object to a list"
  (cl:list 'UnloadLoadNodelet-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'UnloadLoadNodelet)))
  'UnloadLoadNodelet-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'UnloadLoadNodelet)))
  'UnloadLoadNodelet-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UnloadLoadNodelet)))
  "Returns string type for a service object of type '<UnloadLoadNodelet>"
  "ff_msgs/UnloadLoadNodelet")