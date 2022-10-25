; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude GraphState.msg.html

(cl:defclass <GraphState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (child_frame_id
    :reader child_frame_id
    :initarg :child_frame_id
    :type cl:string
    :initform "")
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (velocity
    :reader velocity
    :initarg :velocity
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (gyro_bias
    :reader gyro_bias
    :initarg :gyro_bias
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (accel_bias
    :reader accel_bias
    :initarg :accel_bias
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (cov_diag
    :reader cov_diag
    :initarg :cov_diag
    :type (cl:vector cl:float)
   :initform (cl:make-array 15 :element-type 'cl:float :initial-element 0.0))
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:fixnum
    :initform 0)
   (num_detected_of_features
    :reader num_detected_of_features
    :initarg :num_detected_of_features
    :type cl:integer
    :initform 0)
   (num_detected_ar_features
    :reader num_detected_ar_features
    :initarg :num_detected_ar_features
    :type cl:integer
    :initform 0)
   (num_detected_ml_features
    :reader num_detected_ml_features
    :initarg :num_detected_ml_features
    :type cl:integer
    :initform 0)
   (iterations
    :reader iterations
    :initarg :iterations
    :type cl:integer
    :initform 0)
   (optimization_time
    :reader optimization_time
    :initarg :optimization_time
    :type cl:float
    :initform 0.0)
   (update_time
    :reader update_time
    :initarg :update_time
    :type cl:float
    :initform 0.0)
   (callbacks_time
    :reader callbacks_time
    :initarg :callbacks_time
    :type cl:float
    :initform 0.0)
   (nodelet_runtime
    :reader nodelet_runtime
    :initarg :nodelet_runtime
    :type cl:float
    :initform 0.0)
   (num_factors
    :reader num_factors
    :initarg :num_factors
    :type cl:integer
    :initform 0)
   (num_of_factors
    :reader num_of_factors
    :initarg :num_of_factors
    :type cl:integer
    :initform 0)
   (num_ml_projection_factors
    :reader num_ml_projection_factors
    :initarg :num_ml_projection_factors
    :type cl:integer
    :initform 0)
   (num_ml_pose_factors
    :reader num_ml_pose_factors
    :initarg :num_ml_pose_factors
    :type cl:integer
    :initform 0)
   (num_states
    :reader num_states
    :initarg :num_states
    :type cl:integer
    :initform 0)
   (standstill
    :reader standstill
    :initarg :standstill
    :type cl:boolean
    :initform cl:nil)
   (estimating_bias
    :reader estimating_bias
    :initarg :estimating_bias
    :type cl:boolean
    :initform cl:nil)
   (fan_speed_mode
    :reader fan_speed_mode
    :initarg :fan_speed_mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GraphState (<GraphState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GraphState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GraphState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<GraphState> is deprecated: use ff_msgs-msg:GraphState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GraphState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'child_frame_id-val :lambda-list '(m))
(cl:defmethod child_frame_id-val ((m <GraphState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:child_frame_id-val is deprecated.  Use ff_msgs-msg:child_frame_id instead.")
  (child_frame_id m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <GraphState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:pose-val is deprecated.  Use ff_msgs-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <GraphState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:velocity-val is deprecated.  Use ff_msgs-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'gyro_bias-val :lambda-list '(m))
(cl:defmethod gyro_bias-val ((m <GraphState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:gyro_bias-val is deprecated.  Use ff_msgs-msg:gyro_bias instead.")
  (gyro_bias m))

(cl:ensure-generic-function 'accel_bias-val :lambda-list '(m))
(cl:defmethod accel_bias-val ((m <GraphState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:accel_bias-val is deprecated.  Use ff_msgs-msg:accel_bias instead.")
  (accel_bias m))

(cl:ensure-generic-function 'cov_diag-val :lambda-list '(m))
(cl:defmethod cov_diag-val ((m <GraphState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:cov_diag-val is deprecated.  Use ff_msgs-msg:cov_diag instead.")
  (cov_diag m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <GraphState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:confidence-val is deprecated.  Use ff_msgs-msg:confidence instead.")
  (confidence m))

(cl:ensure-generic-function 'num_detected_of_features-val :lambda-list '(m))
(cl:defmethod num_detected_of_features-val ((m <GraphState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:num_detected_of_features-val is deprecated.  Use ff_msgs-msg:num_detected_of_features instead.")
  (num_detected_of_features m))

(cl:ensure-generic-function 'num_detected_ar_features-val :lambda-list '(m))
(cl:defmethod num_detected_ar_features-val ((m <GraphState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:num_detected_ar_features-val is deprecated.  Use ff_msgs-msg:num_detected_ar_features instead.")
  (num_detected_ar_features m))

(cl:ensure-generic-function 'num_detected_ml_features-val :lambda-list '(m))
(cl:defmethod num_detected_ml_features-val ((m <GraphState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:num_detected_ml_features-val is deprecated.  Use ff_msgs-msg:num_detected_ml_features instead.")
  (num_detected_ml_features m))

(cl:ensure-generic-function 'iterations-val :lambda-list '(m))
(cl:defmethod iterations-val ((m <GraphState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:iterations-val is deprecated.  Use ff_msgs-msg:iterations instead.")
  (iterations m))

(cl:ensure-generic-function 'optimization_time-val :lambda-list '(m))
(cl:defmethod optimization_time-val ((m <GraphState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:optimization_time-val is deprecated.  Use ff_msgs-msg:optimization_time instead.")
  (optimization_time m))

(cl:ensure-generic-function 'update_time-val :lambda-list '(m))
(cl:defmethod update_time-val ((m <GraphState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:update_time-val is deprecated.  Use ff_msgs-msg:update_time instead.")
  (update_time m))

(cl:ensure-generic-function 'callbacks_time-val :lambda-list '(m))
(cl:defmethod callbacks_time-val ((m <GraphState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:callbacks_time-val is deprecated.  Use ff_msgs-msg:callbacks_time instead.")
  (callbacks_time m))

(cl:ensure-generic-function 'nodelet_runtime-val :lambda-list '(m))
(cl:defmethod nodelet_runtime-val ((m <GraphState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:nodelet_runtime-val is deprecated.  Use ff_msgs-msg:nodelet_runtime instead.")
  (nodelet_runtime m))

(cl:ensure-generic-function 'num_factors-val :lambda-list '(m))
(cl:defmethod num_factors-val ((m <GraphState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:num_factors-val is deprecated.  Use ff_msgs-msg:num_factors instead.")
  (num_factors m))

(cl:ensure-generic-function 'num_of_factors-val :lambda-list '(m))
(cl:defmethod num_of_factors-val ((m <GraphState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:num_of_factors-val is deprecated.  Use ff_msgs-msg:num_of_factors instead.")
  (num_of_factors m))

(cl:ensure-generic-function 'num_ml_projection_factors-val :lambda-list '(m))
(cl:defmethod num_ml_projection_factors-val ((m <GraphState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:num_ml_projection_factors-val is deprecated.  Use ff_msgs-msg:num_ml_projection_factors instead.")
  (num_ml_projection_factors m))

(cl:ensure-generic-function 'num_ml_pose_factors-val :lambda-list '(m))
(cl:defmethod num_ml_pose_factors-val ((m <GraphState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:num_ml_pose_factors-val is deprecated.  Use ff_msgs-msg:num_ml_pose_factors instead.")
  (num_ml_pose_factors m))

(cl:ensure-generic-function 'num_states-val :lambda-list '(m))
(cl:defmethod num_states-val ((m <GraphState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:num_states-val is deprecated.  Use ff_msgs-msg:num_states instead.")
  (num_states m))

(cl:ensure-generic-function 'standstill-val :lambda-list '(m))
(cl:defmethod standstill-val ((m <GraphState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:standstill-val is deprecated.  Use ff_msgs-msg:standstill instead.")
  (standstill m))

(cl:ensure-generic-function 'estimating_bias-val :lambda-list '(m))
(cl:defmethod estimating_bias-val ((m <GraphState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:estimating_bias-val is deprecated.  Use ff_msgs-msg:estimating_bias instead.")
  (estimating_bias m))

(cl:ensure-generic-function 'fan_speed_mode-val :lambda-list '(m))
(cl:defmethod fan_speed_mode-val ((m <GraphState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:fan_speed_mode-val is deprecated.  Use ff_msgs-msg:fan_speed_mode instead.")
  (fan_speed_mode m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GraphState>)))
    "Constants for message type '<GraphState>"
  '((:CONFIDENCE_GOOD . 0)
    (:CONFIDENCE_POOR . 1)
    (:CONFIDENCE_LOST . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GraphState)))
    "Constants for message type 'GraphState"
  '((:CONFIDENCE_GOOD . 0)
    (:CONFIDENCE_POOR . 1)
    (:CONFIDENCE_LOST . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GraphState>) ostream)
  "Serializes a message object of type '<GraphState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'child_frame_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'child_frame_id))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'gyro_bias) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'accel_bias) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'cov_diag))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'confidence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_detected_of_features)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_detected_of_features)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_detected_of_features)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_detected_of_features)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_detected_ar_features)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_detected_ar_features)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_detected_ar_features)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_detected_ar_features)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_detected_ml_features)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_detected_ml_features)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_detected_ml_features)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_detected_ml_features)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'iterations)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'iterations)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'iterations)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'iterations)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'optimization_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'update_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'callbacks_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'nodelet_runtime))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_factors)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_factors)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_factors)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_factors)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_of_factors)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_of_factors)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_of_factors)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_of_factors)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_ml_projection_factors)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_ml_projection_factors)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_ml_projection_factors)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_ml_projection_factors)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_ml_pose_factors)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_ml_pose_factors)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_ml_pose_factors)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_ml_pose_factors)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_states)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_states)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_states)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_states)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'standstill) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'estimating_bias) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fan_speed_mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GraphState>) istream)
  "Deserializes a message object of type '<GraphState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'child_frame_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'child_frame_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'gyro_bias) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'accel_bias) istream)
  (cl:setf (cl:slot-value msg 'cov_diag) (cl:make-array 15))
  (cl:let ((vals (cl:slot-value msg 'cov_diag)))
    (cl:dotimes (i 15)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'confidence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_detected_of_features)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_detected_of_features)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_detected_of_features)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_detected_of_features)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_detected_ar_features)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_detected_ar_features)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_detected_ar_features)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_detected_ar_features)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_detected_ml_features)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_detected_ml_features)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_detected_ml_features)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_detected_ml_features)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'iterations)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'iterations)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'iterations)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'iterations)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'optimization_time) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'update_time) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'callbacks_time) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'nodelet_runtime) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_factors)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_factors)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_factors)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_factors)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_of_factors)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_of_factors)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_of_factors)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_of_factors)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_ml_projection_factors)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_ml_projection_factors)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_ml_projection_factors)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_ml_projection_factors)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_ml_pose_factors)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_ml_pose_factors)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_ml_pose_factors)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_ml_pose_factors)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_states)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_states)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_states)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_states)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'standstill) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'estimating_bias) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fan_speed_mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GraphState>)))
  "Returns string type for a message object of type '<GraphState>"
  "ff_msgs/GraphState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GraphState)))
  "Returns string type for a message object of type 'GraphState"
  "ff_msgs/GraphState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GraphState>)))
  "Returns md5sum for a message object of type '<GraphState>"
  "d0020fbc20fe81214e0f3f2b41dd4c22")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GraphState)))
  "Returns md5sum for a message object of type 'GraphState"
  "d0020fbc20fe81214e0f3f2b41dd4c22")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GraphState>)))
  "Returns full string definition for message of type '<GraphState>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%~%std_msgs/Header header # header with timestamp~%string child_frame_id # frame ID~%# State Estimates~%geometry_msgs/Pose pose # world_T_body ~%geometry_msgs/Vector3 velocity # body velocity~%geometry_msgs/Vector3 gyro_bias # estimated gyro bias~%geometry_msgs/Vector3 accel_bias # estimated accel bias~%# Covariances/Confidences~%# covariance diagonal. 1-3 orientation, 4-6 gyro bias, 7-9 velocity, 10-12 accel bias, 13-15 position~%float32[15] cov_diag~%# confidence in estimate. 0 is good, 1 is a bit confused, 2 is lost~%uint8 confidence~%uint8 CONFIDENCE_GOOD = 0	# Tracking well~%uint8 CONFIDENCE_POOR = 1	# Tracking poorly~%uint8 CONFIDENCE_LOST = 2	# We are lost~%# Stats~%uint32 num_detected_of_features  ~%uint32 num_detected_ar_features ~%uint32 num_detected_ml_features ~%uint32 iterations # Optimization iterations~%float32 optimization_time~%float32 update_time # Include optimization_time and other operations to add data to graph~%float32 callbacks_time # Includes processing msgs and their callbacks~%float32 nodelet_runtime # Total runtime of nodelet iteration.  Includes update and callback time~%uint32 num_factors~%uint32 num_of_factors~%uint32 num_ml_projection_factors~%uint32 num_ml_pose_factors~%uint32 num_states~%# Status~%bool standstill~%bool estimating_bias # Are we busy estimating the bias?~%uint8 fan_speed_mode # Used for imu filtering~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GraphState)))
  "Returns full string definition for message of type 'GraphState"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%~%std_msgs/Header header # header with timestamp~%string child_frame_id # frame ID~%# State Estimates~%geometry_msgs/Pose pose # world_T_body ~%geometry_msgs/Vector3 velocity # body velocity~%geometry_msgs/Vector3 gyro_bias # estimated gyro bias~%geometry_msgs/Vector3 accel_bias # estimated accel bias~%# Covariances/Confidences~%# covariance diagonal. 1-3 orientation, 4-6 gyro bias, 7-9 velocity, 10-12 accel bias, 13-15 position~%float32[15] cov_diag~%# confidence in estimate. 0 is good, 1 is a bit confused, 2 is lost~%uint8 confidence~%uint8 CONFIDENCE_GOOD = 0	# Tracking well~%uint8 CONFIDENCE_POOR = 1	# Tracking poorly~%uint8 CONFIDENCE_LOST = 2	# We are lost~%# Stats~%uint32 num_detected_of_features  ~%uint32 num_detected_ar_features ~%uint32 num_detected_ml_features ~%uint32 iterations # Optimization iterations~%float32 optimization_time~%float32 update_time # Include optimization_time and other operations to add data to graph~%float32 callbacks_time # Includes processing msgs and their callbacks~%float32 nodelet_runtime # Total runtime of nodelet iteration.  Includes update and callback time~%uint32 num_factors~%uint32 num_of_factors~%uint32 num_ml_projection_factors~%uint32 num_ml_pose_factors~%uint32 num_states~%# Status~%bool standstill~%bool estimating_bias # Are we busy estimating the bias?~%uint8 fan_speed_mode # Used for imu filtering~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GraphState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'child_frame_id))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'gyro_bias))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'accel_bias))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'cov_diag) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GraphState>))
  "Converts a ROS message object to a list"
  (cl:list 'GraphState
    (cl:cons ':header (header msg))
    (cl:cons ':child_frame_id (child_frame_id msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':gyro_bias (gyro_bias msg))
    (cl:cons ':accel_bias (accel_bias msg))
    (cl:cons ':cov_diag (cov_diag msg))
    (cl:cons ':confidence (confidence msg))
    (cl:cons ':num_detected_of_features (num_detected_of_features msg))
    (cl:cons ':num_detected_ar_features (num_detected_ar_features msg))
    (cl:cons ':num_detected_ml_features (num_detected_ml_features msg))
    (cl:cons ':iterations (iterations msg))
    (cl:cons ':optimization_time (optimization_time msg))
    (cl:cons ':update_time (update_time msg))
    (cl:cons ':callbacks_time (callbacks_time msg))
    (cl:cons ':nodelet_runtime (nodelet_runtime msg))
    (cl:cons ':num_factors (num_factors msg))
    (cl:cons ':num_of_factors (num_of_factors msg))
    (cl:cons ':num_ml_projection_factors (num_ml_projection_factors msg))
    (cl:cons ':num_ml_pose_factors (num_ml_pose_factors msg))
    (cl:cons ':num_states (num_states msg))
    (cl:cons ':standstill (standstill msg))
    (cl:cons ':estimating_bias (estimating_bias msg))
    (cl:cons ':fan_speed_mode (fan_speed_mode msg))
))
