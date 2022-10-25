; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude AgentStateStamped.msg.html

(cl:defclass <AgentStateStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (operating_state
    :reader operating_state
    :initarg :operating_state
    :type ff_msgs-msg:OpState
    :initform (cl:make-instance 'ff_msgs-msg:OpState))
   (plan_execution_state
    :reader plan_execution_state
    :initarg :plan_execution_state
    :type ff_msgs-msg:ExecState
    :initform (cl:make-instance 'ff_msgs-msg:ExecState))
   (guest_science_state
    :reader guest_science_state
    :initarg :guest_science_state
    :type ff_msgs-msg:ExecState
    :initform (cl:make-instance 'ff_msgs-msg:ExecState))
   (mobility_state
    :reader mobility_state
    :initarg :mobility_state
    :type ff_msgs-msg:MobilityState
    :initform (cl:make-instance 'ff_msgs-msg:MobilityState))
   (proximity
    :reader proximity
    :initarg :proximity
    :type cl:float
    :initform 0.0)
   (profile_name
    :reader profile_name
    :initarg :profile_name
    :type cl:string
    :initform "")
   (flight_mode
    :reader flight_mode
    :initarg :flight_mode
    :type cl:string
    :initform "")
   (target_linear_velocity
    :reader target_linear_velocity
    :initarg :target_linear_velocity
    :type cl:float
    :initform 0.0)
   (target_linear_accel
    :reader target_linear_accel
    :initarg :target_linear_accel
    :type cl:float
    :initform 0.0)
   (target_angular_velocity
    :reader target_angular_velocity
    :initarg :target_angular_velocity
    :type cl:float
    :initform 0.0)
   (target_angular_accel
    :reader target_angular_accel
    :initarg :target_angular_accel
    :type cl:float
    :initform 0.0)
   (collision_distance
    :reader collision_distance
    :initarg :collision_distance
    :type cl:float
    :initform 0.0)
   (holonomic_enabled
    :reader holonomic_enabled
    :initarg :holonomic_enabled
    :type cl:boolean
    :initform cl:nil)
   (check_obstacles
    :reader check_obstacles
    :initarg :check_obstacles
    :type cl:boolean
    :initform cl:nil)
   (check_zones
    :reader check_zones
    :initarg :check_zones
    :type cl:boolean
    :initform cl:nil)
   (auto_return_enabled
    :reader auto_return_enabled
    :initarg :auto_return_enabled
    :type cl:boolean
    :initform cl:nil)
   (immediate_enabled
    :reader immediate_enabled
    :initarg :immediate_enabled
    :type cl:boolean
    :initform cl:nil)
   (planner
    :reader planner
    :initarg :planner
    :type cl:string
    :initform "")
   (replanning_enabled
    :reader replanning_enabled
    :initarg :replanning_enabled
    :type cl:boolean
    :initform cl:nil)
   (world
    :reader world
    :initarg :world
    :type cl:string
    :initform "")
   (boot_time
    :reader boot_time
    :initarg :boot_time
    :type cl:integer
    :initform 0))
)

(cl:defclass AgentStateStamped (<AgentStateStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AgentStateStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AgentStateStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<AgentStateStamped> is deprecated: use ff_msgs-msg:AgentStateStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AgentStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'operating_state-val :lambda-list '(m))
(cl:defmethod operating_state-val ((m <AgentStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:operating_state-val is deprecated.  Use ff_msgs-msg:operating_state instead.")
  (operating_state m))

(cl:ensure-generic-function 'plan_execution_state-val :lambda-list '(m))
(cl:defmethod plan_execution_state-val ((m <AgentStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:plan_execution_state-val is deprecated.  Use ff_msgs-msg:plan_execution_state instead.")
  (plan_execution_state m))

(cl:ensure-generic-function 'guest_science_state-val :lambda-list '(m))
(cl:defmethod guest_science_state-val ((m <AgentStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:guest_science_state-val is deprecated.  Use ff_msgs-msg:guest_science_state instead.")
  (guest_science_state m))

(cl:ensure-generic-function 'mobility_state-val :lambda-list '(m))
(cl:defmethod mobility_state-val ((m <AgentStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:mobility_state-val is deprecated.  Use ff_msgs-msg:mobility_state instead.")
  (mobility_state m))

(cl:ensure-generic-function 'proximity-val :lambda-list '(m))
(cl:defmethod proximity-val ((m <AgentStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:proximity-val is deprecated.  Use ff_msgs-msg:proximity instead.")
  (proximity m))

(cl:ensure-generic-function 'profile_name-val :lambda-list '(m))
(cl:defmethod profile_name-val ((m <AgentStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:profile_name-val is deprecated.  Use ff_msgs-msg:profile_name instead.")
  (profile_name m))

(cl:ensure-generic-function 'flight_mode-val :lambda-list '(m))
(cl:defmethod flight_mode-val ((m <AgentStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:flight_mode-val is deprecated.  Use ff_msgs-msg:flight_mode instead.")
  (flight_mode m))

(cl:ensure-generic-function 'target_linear_velocity-val :lambda-list '(m))
(cl:defmethod target_linear_velocity-val ((m <AgentStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:target_linear_velocity-val is deprecated.  Use ff_msgs-msg:target_linear_velocity instead.")
  (target_linear_velocity m))

(cl:ensure-generic-function 'target_linear_accel-val :lambda-list '(m))
(cl:defmethod target_linear_accel-val ((m <AgentStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:target_linear_accel-val is deprecated.  Use ff_msgs-msg:target_linear_accel instead.")
  (target_linear_accel m))

(cl:ensure-generic-function 'target_angular_velocity-val :lambda-list '(m))
(cl:defmethod target_angular_velocity-val ((m <AgentStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:target_angular_velocity-val is deprecated.  Use ff_msgs-msg:target_angular_velocity instead.")
  (target_angular_velocity m))

(cl:ensure-generic-function 'target_angular_accel-val :lambda-list '(m))
(cl:defmethod target_angular_accel-val ((m <AgentStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:target_angular_accel-val is deprecated.  Use ff_msgs-msg:target_angular_accel instead.")
  (target_angular_accel m))

(cl:ensure-generic-function 'collision_distance-val :lambda-list '(m))
(cl:defmethod collision_distance-val ((m <AgentStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:collision_distance-val is deprecated.  Use ff_msgs-msg:collision_distance instead.")
  (collision_distance m))

(cl:ensure-generic-function 'holonomic_enabled-val :lambda-list '(m))
(cl:defmethod holonomic_enabled-val ((m <AgentStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:holonomic_enabled-val is deprecated.  Use ff_msgs-msg:holonomic_enabled instead.")
  (holonomic_enabled m))

(cl:ensure-generic-function 'check_obstacles-val :lambda-list '(m))
(cl:defmethod check_obstacles-val ((m <AgentStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:check_obstacles-val is deprecated.  Use ff_msgs-msg:check_obstacles instead.")
  (check_obstacles m))

(cl:ensure-generic-function 'check_zones-val :lambda-list '(m))
(cl:defmethod check_zones-val ((m <AgentStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:check_zones-val is deprecated.  Use ff_msgs-msg:check_zones instead.")
  (check_zones m))

(cl:ensure-generic-function 'auto_return_enabled-val :lambda-list '(m))
(cl:defmethod auto_return_enabled-val ((m <AgentStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:auto_return_enabled-val is deprecated.  Use ff_msgs-msg:auto_return_enabled instead.")
  (auto_return_enabled m))

(cl:ensure-generic-function 'immediate_enabled-val :lambda-list '(m))
(cl:defmethod immediate_enabled-val ((m <AgentStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:immediate_enabled-val is deprecated.  Use ff_msgs-msg:immediate_enabled instead.")
  (immediate_enabled m))

(cl:ensure-generic-function 'planner-val :lambda-list '(m))
(cl:defmethod planner-val ((m <AgentStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:planner-val is deprecated.  Use ff_msgs-msg:planner instead.")
  (planner m))

(cl:ensure-generic-function 'replanning_enabled-val :lambda-list '(m))
(cl:defmethod replanning_enabled-val ((m <AgentStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:replanning_enabled-val is deprecated.  Use ff_msgs-msg:replanning_enabled instead.")
  (replanning_enabled m))

(cl:ensure-generic-function 'world-val :lambda-list '(m))
(cl:defmethod world-val ((m <AgentStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:world-val is deprecated.  Use ff_msgs-msg:world instead.")
  (world m))

(cl:ensure-generic-function 'boot_time-val :lambda-list '(m))
(cl:defmethod boot_time-val ((m <AgentStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:boot_time-val is deprecated.  Use ff_msgs-msg:boot_time instead.")
  (boot_time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AgentStateStamped>) ostream)
  "Serializes a message object of type '<AgentStateStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'operating_state) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'plan_execution_state) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'guest_science_state) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'mobility_state) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'proximity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'profile_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'profile_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'flight_mode))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'flight_mode))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'target_linear_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'target_linear_accel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'target_angular_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'target_angular_accel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'collision_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'holonomic_enabled) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'check_obstacles) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'check_zones) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'auto_return_enabled) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'immediate_enabled) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'planner))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'planner))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'replanning_enabled) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'world))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'world))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'boot_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'boot_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'boot_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'boot_time)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AgentStateStamped>) istream)
  "Deserializes a message object of type '<AgentStateStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'operating_state) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'plan_execution_state) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'guest_science_state) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'mobility_state) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'proximity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'profile_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'profile_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'flight_mode) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'flight_mode) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_linear_velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_linear_accel) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_angular_velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_angular_accel) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'collision_distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'holonomic_enabled) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'check_obstacles) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'check_zones) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'auto_return_enabled) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'immediate_enabled) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'planner) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'planner) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'replanning_enabled) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'world) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'world) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'boot_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'boot_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'boot_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'boot_time)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AgentStateStamped>)))
  "Returns string type for a message object of type '<AgentStateStamped>"
  "ff_msgs/AgentStateStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AgentStateStamped)))
  "Returns string type for a message object of type 'AgentStateStamped"
  "ff_msgs/AgentStateStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AgentStateStamped>)))
  "Returns md5sum for a message object of type '<AgentStateStamped>"
  "156487b23e377e3a1dc7ef079f0e327d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AgentStateStamped)))
  "Returns md5sum for a message object of type 'AgentStateStamped"
  "156487b23e377e3a1dc7ef079f0e327d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AgentStateStamped>)))
  "Returns full string definition for message of type '<AgentStateStamped>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# State of the Astrobee, based off of rapid::ext::astrobee::AgentState~%~%# Header with timestamp~%std_msgs/Header header~%~%# Operating state of the Astrobee~%ff_msgs/OpState operating_state~%~%# Plan execution state. State is idle when there is no plan to be executed. Once~%# a plan is uploaded, the state change to paused. Upon a run plan command, the~%# state will change to executing. If a stop or pause plan command is received or~%# a fault occurs, the state will be set to pause. Once the plan is completed,~%# the state will go back to idle~%ff_msgs/ExecState plan_execution_state~%~%# Guest science state. If a primary guest science apk is started, the state~%# will go from idle to executing. Once the primarty apk is finished, the state~%# will go back to idle~%ff_msgs/ExecState guest_science_state~%~%# Mobility state of the Astrobee~%ff_msgs/MobilityState mobility_state~%~%# Proximity to the dock when docking and undocking. Proximity to a handrail when~%# perching or unperching. 0 the rest of the time.~%float32 proximity~%~%# Name of profile configuration, i.e. Nominal, IgnoreObstacles, Faceforward,~%# Quiet, etc. Profiles specify stuff like target velocity and acceleration,~%# collision distance, etc.~%string profile_name~%~%#Defines GN&C gains, hard limits, tolerances, etc.~%string flight_mode~%~%# Maximum linear velocity to target while translating~%float32 target_linear_velocity~%# Maximum linear acceleration to target while translating~%float32 target_linear_accel~%# Maximum angular velocity to target while rotating~%float32 target_angular_velocity~%# Maximum angular acceleration to target while rotating~%float32 target_angular_accel~%# Minimum distance margin to maintain away from obstacles~%float32 collision_distance~%~%# Specifies whether the Astrobee is allowed to fly blind i.e. not faceforward~%bool holonomic_enabled~%~%# Specifies whether the Astrobee is checking for obstacles~%bool check_obstacles~%~%# Specifies whether the Astrobee is checking the keepin and keepout zones~%bool check_zones~%~%# Specifies whether the Astrobee is allowed to auto return. Please note,~%# Astrobee will only use this flags when deciding if it should auto return. If~%# the astrobee gets a command to auto return from the operator or guest science,~%# it will auto return without checking this flag~%bool auto_return_enabled~%~%# Specifies whether the choreographer should execute a segment immediately or~%# based on the time stamp in the segement~%bool immediate_enabled~%~%# Specifies the current planner being used~%string planner~%~%# Specifies whether re-planning is allowed~%bool replanning_enabled~%~%# Specifies the current world being used~%string world~%~%# Number of seconds since Unix Epoch~%uint32 boot_time~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/OpState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Operating States, based off of the enumeration constants~%# in rapid::ext::astrobee::AgentState.~%#~%# *MUST* be kept in sync with the DDS IDL file in astrobee_common~%~%uint8 READY            = 0  # Freeflyer is ready to accept commands~%uint8 PLAN_EXECUTION   = 1  # Freeflyer is executing a plan~%uint8 TELEOPERATION    = 2  # Freeflyer is executing a teleop command~%uint8 AUTO_RETURN      = 3  # Freeflyer is autonomously returning to the dock~%# The freeflyer is either executing a fault response or there is a fault~%# occurring in the system that prevents the freeflyer from moving~%uint8 FAULT            = 4~%~%# Operating state~%uint8 state~%~%================================================================================~%MSG: ff_msgs/ExecState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Execution States, based off of the enumeration constants in~%# rapid::ext::astrobee::AgentState~%#~%# *MUST* be kept in sync with the DDS IDL file in astrobee_common~%~%uint8 IDLE      = 0   # Process is idle~%uint8 EXECUTING = 1   # Process is executing~%uint8 PAUSED    = 2   # Process is paused~%uint8 ERROR     = 3   # Process encountered an error~%~%# Execution state~%uint8 state~%~%================================================================================~%MSG: ff_msgs/MobilityState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Mobility states, based off the enumeration constants in~%# rapid::ext::astrobee::AgentState~%#~%# *MUST* be kept in sync with the DDS IDL file in astrobee_common~%~%uint8 DRIFTING        = 0   # Astrobee's propulsion is off~%uint8 STOPPING        = 1   # Astrobee is either stopping or stopped~%uint8 FLYING          = 2   # Astrobee is flying~%uint8 DOCKING         = 3   # Astrobee is either docking or undocking~%uint8 PERCHING        = 4   # Astrobee is either perching or unperching~%~%# Mobility state~%uint8 state~%~%# Specifies the progress of the action. For docking, this value can be N to -N~%# where N through 1 specifies the progress of a docking action, 0 is docked, and~%# -1 through -N specifies the process of an undocking action. For stopping, this~%# value is either 1 or 0 where 1 means the robot is coming to a stop and 0 means~%# the robot is stopped. For perching, this value can be N to -N where N through~%# 1 specifies the progress of a perching action, 0 is perched, and -1 through~%# -N specifies the process of an unperching action.~%int32 sub_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AgentStateStamped)))
  "Returns full string definition for message of type 'AgentStateStamped"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# State of the Astrobee, based off of rapid::ext::astrobee::AgentState~%~%# Header with timestamp~%std_msgs/Header header~%~%# Operating state of the Astrobee~%ff_msgs/OpState operating_state~%~%# Plan execution state. State is idle when there is no plan to be executed. Once~%# a plan is uploaded, the state change to paused. Upon a run plan command, the~%# state will change to executing. If a stop or pause plan command is received or~%# a fault occurs, the state will be set to pause. Once the plan is completed,~%# the state will go back to idle~%ff_msgs/ExecState plan_execution_state~%~%# Guest science state. If a primary guest science apk is started, the state~%# will go from idle to executing. Once the primarty apk is finished, the state~%# will go back to idle~%ff_msgs/ExecState guest_science_state~%~%# Mobility state of the Astrobee~%ff_msgs/MobilityState mobility_state~%~%# Proximity to the dock when docking and undocking. Proximity to a handrail when~%# perching or unperching. 0 the rest of the time.~%float32 proximity~%~%# Name of profile configuration, i.e. Nominal, IgnoreObstacles, Faceforward,~%# Quiet, etc. Profiles specify stuff like target velocity and acceleration,~%# collision distance, etc.~%string profile_name~%~%#Defines GN&C gains, hard limits, tolerances, etc.~%string flight_mode~%~%# Maximum linear velocity to target while translating~%float32 target_linear_velocity~%# Maximum linear acceleration to target while translating~%float32 target_linear_accel~%# Maximum angular velocity to target while rotating~%float32 target_angular_velocity~%# Maximum angular acceleration to target while rotating~%float32 target_angular_accel~%# Minimum distance margin to maintain away from obstacles~%float32 collision_distance~%~%# Specifies whether the Astrobee is allowed to fly blind i.e. not faceforward~%bool holonomic_enabled~%~%# Specifies whether the Astrobee is checking for obstacles~%bool check_obstacles~%~%# Specifies whether the Astrobee is checking the keepin and keepout zones~%bool check_zones~%~%# Specifies whether the Astrobee is allowed to auto return. Please note,~%# Astrobee will only use this flags when deciding if it should auto return. If~%# the astrobee gets a command to auto return from the operator or guest science,~%# it will auto return without checking this flag~%bool auto_return_enabled~%~%# Specifies whether the choreographer should execute a segment immediately or~%# based on the time stamp in the segement~%bool immediate_enabled~%~%# Specifies the current planner being used~%string planner~%~%# Specifies whether re-planning is allowed~%bool replanning_enabled~%~%# Specifies the current world being used~%string world~%~%# Number of seconds since Unix Epoch~%uint32 boot_time~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/OpState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Operating States, based off of the enumeration constants~%# in rapid::ext::astrobee::AgentState.~%#~%# *MUST* be kept in sync with the DDS IDL file in astrobee_common~%~%uint8 READY            = 0  # Freeflyer is ready to accept commands~%uint8 PLAN_EXECUTION   = 1  # Freeflyer is executing a plan~%uint8 TELEOPERATION    = 2  # Freeflyer is executing a teleop command~%uint8 AUTO_RETURN      = 3  # Freeflyer is autonomously returning to the dock~%# The freeflyer is either executing a fault response or there is a fault~%# occurring in the system that prevents the freeflyer from moving~%uint8 FAULT            = 4~%~%# Operating state~%uint8 state~%~%================================================================================~%MSG: ff_msgs/ExecState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Execution States, based off of the enumeration constants in~%# rapid::ext::astrobee::AgentState~%#~%# *MUST* be kept in sync with the DDS IDL file in astrobee_common~%~%uint8 IDLE      = 0   # Process is idle~%uint8 EXECUTING = 1   # Process is executing~%uint8 PAUSED    = 2   # Process is paused~%uint8 ERROR     = 3   # Process encountered an error~%~%# Execution state~%uint8 state~%~%================================================================================~%MSG: ff_msgs/MobilityState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Mobility states, based off the enumeration constants in~%# rapid::ext::astrobee::AgentState~%#~%# *MUST* be kept in sync with the DDS IDL file in astrobee_common~%~%uint8 DRIFTING        = 0   # Astrobee's propulsion is off~%uint8 STOPPING        = 1   # Astrobee is either stopping or stopped~%uint8 FLYING          = 2   # Astrobee is flying~%uint8 DOCKING         = 3   # Astrobee is either docking or undocking~%uint8 PERCHING        = 4   # Astrobee is either perching or unperching~%~%# Mobility state~%uint8 state~%~%# Specifies the progress of the action. For docking, this value can be N to -N~%# where N through 1 specifies the progress of a docking action, 0 is docked, and~%# -1 through -N specifies the process of an undocking action. For stopping, this~%# value is either 1 or 0 where 1 means the robot is coming to a stop and 0 means~%# the robot is stopped. For perching, this value can be N to -N where N through~%# 1 specifies the progress of a perching action, 0 is perched, and -1 through~%# -N specifies the process of an unperching action.~%int32 sub_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AgentStateStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'operating_state))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'plan_execution_state))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'guest_science_state))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'mobility_state))
     4
     4 (cl:length (cl:slot-value msg 'profile_name))
     4 (cl:length (cl:slot-value msg 'flight_mode))
     4
     4
     4
     4
     4
     1
     1
     1
     1
     1
     4 (cl:length (cl:slot-value msg 'planner))
     1
     4 (cl:length (cl:slot-value msg 'world))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AgentStateStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'AgentStateStamped
    (cl:cons ':header (header msg))
    (cl:cons ':operating_state (operating_state msg))
    (cl:cons ':plan_execution_state (plan_execution_state msg))
    (cl:cons ':guest_science_state (guest_science_state msg))
    (cl:cons ':mobility_state (mobility_state msg))
    (cl:cons ':proximity (proximity msg))
    (cl:cons ':profile_name (profile_name msg))
    (cl:cons ':flight_mode (flight_mode msg))
    (cl:cons ':target_linear_velocity (target_linear_velocity msg))
    (cl:cons ':target_linear_accel (target_linear_accel msg))
    (cl:cons ':target_angular_velocity (target_angular_velocity msg))
    (cl:cons ':target_angular_accel (target_angular_accel msg))
    (cl:cons ':collision_distance (collision_distance msg))
    (cl:cons ':holonomic_enabled (holonomic_enabled msg))
    (cl:cons ':check_obstacles (check_obstacles msg))
    (cl:cons ':check_zones (check_zones msg))
    (cl:cons ':auto_return_enabled (auto_return_enabled msg))
    (cl:cons ':immediate_enabled (immediate_enabled msg))
    (cl:cons ':planner (planner msg))
    (cl:cons ':replanning_enabled (replanning_enabled msg))
    (cl:cons ':world (world msg))
    (cl:cons ':boot_time (boot_time msg))
))
