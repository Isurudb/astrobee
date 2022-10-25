;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::GraphState)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'GraphState (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::GRAPHSTATE")
  (make-package "FF_MSGS::GRAPHSTATE"))

(in-package "ROS")
;;//! \htmlinclude GraphState.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*CONFIDENCE_GOOD*" (find-package "FF_MSGS::GRAPHSTATE"))
(shadow '*CONFIDENCE_GOOD* (find-package "FF_MSGS::GRAPHSTATE"))
(defconstant ff_msgs::GraphState::*CONFIDENCE_GOOD* 0)
(intern "*CONFIDENCE_POOR*" (find-package "FF_MSGS::GRAPHSTATE"))
(shadow '*CONFIDENCE_POOR* (find-package "FF_MSGS::GRAPHSTATE"))
(defconstant ff_msgs::GraphState::*CONFIDENCE_POOR* 1)
(intern "*CONFIDENCE_LOST*" (find-package "FF_MSGS::GRAPHSTATE"))
(shadow '*CONFIDENCE_LOST* (find-package "FF_MSGS::GRAPHSTATE"))
(defconstant ff_msgs::GraphState::*CONFIDENCE_LOST* 2)
(defclass ff_msgs::GraphState
  :super ros::object
  :slots (_header _child_frame_id _pose _velocity _gyro_bias _accel_bias _cov_diag _confidence _num_detected_of_features _num_detected_ar_features _num_detected_ml_features _iterations _optimization_time _update_time _callbacks_time _nodelet_runtime _num_factors _num_of_factors _num_ml_projection_factors _num_ml_pose_factors _num_states _standstill _estimating_bias _fan_speed_mode ))

(defmethod ff_msgs::GraphState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:child_frame_id __child_frame_id) "")
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:velocity __velocity) (instance geometry_msgs::Vector3 :init))
    ((:gyro_bias __gyro_bias) (instance geometry_msgs::Vector3 :init))
    ((:accel_bias __accel_bias) (instance geometry_msgs::Vector3 :init))
    ((:cov_diag __cov_diag) (make-array 15 :initial-element 0.0 :element-type :float))
    ((:confidence __confidence) 0)
    ((:num_detected_of_features __num_detected_of_features) 0)
    ((:num_detected_ar_features __num_detected_ar_features) 0)
    ((:num_detected_ml_features __num_detected_ml_features) 0)
    ((:iterations __iterations) 0)
    ((:optimization_time __optimization_time) 0.0)
    ((:update_time __update_time) 0.0)
    ((:callbacks_time __callbacks_time) 0.0)
    ((:nodelet_runtime __nodelet_runtime) 0.0)
    ((:num_factors __num_factors) 0)
    ((:num_of_factors __num_of_factors) 0)
    ((:num_ml_projection_factors __num_ml_projection_factors) 0)
    ((:num_ml_pose_factors __num_ml_pose_factors) 0)
    ((:num_states __num_states) 0)
    ((:standstill __standstill) nil)
    ((:estimating_bias __estimating_bias) nil)
    ((:fan_speed_mode __fan_speed_mode) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _child_frame_id (string __child_frame_id))
   (setq _pose __pose)
   (setq _velocity __velocity)
   (setq _gyro_bias __gyro_bias)
   (setq _accel_bias __accel_bias)
   (setq _cov_diag __cov_diag)
   (setq _confidence (round __confidence))
   (setq _num_detected_of_features (round __num_detected_of_features))
   (setq _num_detected_ar_features (round __num_detected_ar_features))
   (setq _num_detected_ml_features (round __num_detected_ml_features))
   (setq _iterations (round __iterations))
   (setq _optimization_time (float __optimization_time))
   (setq _update_time (float __update_time))
   (setq _callbacks_time (float __callbacks_time))
   (setq _nodelet_runtime (float __nodelet_runtime))
   (setq _num_factors (round __num_factors))
   (setq _num_of_factors (round __num_of_factors))
   (setq _num_ml_projection_factors (round __num_ml_projection_factors))
   (setq _num_ml_pose_factors (round __num_ml_pose_factors))
   (setq _num_states (round __num_states))
   (setq _standstill __standstill)
   (setq _estimating_bias __estimating_bias)
   (setq _fan_speed_mode (round __fan_speed_mode))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:child_frame_id
   (&optional __child_frame_id)
   (if __child_frame_id (setq _child_frame_id __child_frame_id)) _child_frame_id)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:velocity
   (&rest __velocity)
   (if (keywordp (car __velocity))
       (send* _velocity __velocity)
     (progn
       (if __velocity (setq _velocity (car __velocity)))
       _velocity)))
  (:gyro_bias
   (&rest __gyro_bias)
   (if (keywordp (car __gyro_bias))
       (send* _gyro_bias __gyro_bias)
     (progn
       (if __gyro_bias (setq _gyro_bias (car __gyro_bias)))
       _gyro_bias)))
  (:accel_bias
   (&rest __accel_bias)
   (if (keywordp (car __accel_bias))
       (send* _accel_bias __accel_bias)
     (progn
       (if __accel_bias (setq _accel_bias (car __accel_bias)))
       _accel_bias)))
  (:cov_diag
   (&optional __cov_diag)
   (if __cov_diag (setq _cov_diag __cov_diag)) _cov_diag)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:num_detected_of_features
   (&optional __num_detected_of_features)
   (if __num_detected_of_features (setq _num_detected_of_features __num_detected_of_features)) _num_detected_of_features)
  (:num_detected_ar_features
   (&optional __num_detected_ar_features)
   (if __num_detected_ar_features (setq _num_detected_ar_features __num_detected_ar_features)) _num_detected_ar_features)
  (:num_detected_ml_features
   (&optional __num_detected_ml_features)
   (if __num_detected_ml_features (setq _num_detected_ml_features __num_detected_ml_features)) _num_detected_ml_features)
  (:iterations
   (&optional __iterations)
   (if __iterations (setq _iterations __iterations)) _iterations)
  (:optimization_time
   (&optional __optimization_time)
   (if __optimization_time (setq _optimization_time __optimization_time)) _optimization_time)
  (:update_time
   (&optional __update_time)
   (if __update_time (setq _update_time __update_time)) _update_time)
  (:callbacks_time
   (&optional __callbacks_time)
   (if __callbacks_time (setq _callbacks_time __callbacks_time)) _callbacks_time)
  (:nodelet_runtime
   (&optional __nodelet_runtime)
   (if __nodelet_runtime (setq _nodelet_runtime __nodelet_runtime)) _nodelet_runtime)
  (:num_factors
   (&optional __num_factors)
   (if __num_factors (setq _num_factors __num_factors)) _num_factors)
  (:num_of_factors
   (&optional __num_of_factors)
   (if __num_of_factors (setq _num_of_factors __num_of_factors)) _num_of_factors)
  (:num_ml_projection_factors
   (&optional __num_ml_projection_factors)
   (if __num_ml_projection_factors (setq _num_ml_projection_factors __num_ml_projection_factors)) _num_ml_projection_factors)
  (:num_ml_pose_factors
   (&optional __num_ml_pose_factors)
   (if __num_ml_pose_factors (setq _num_ml_pose_factors __num_ml_pose_factors)) _num_ml_pose_factors)
  (:num_states
   (&optional __num_states)
   (if __num_states (setq _num_states __num_states)) _num_states)
  (:standstill
   (&optional __standstill)
   (if __standstill (setq _standstill __standstill)) _standstill)
  (:estimating_bias
   (&optional __estimating_bias)
   (if __estimating_bias (setq _estimating_bias __estimating_bias)) _estimating_bias)
  (:fan_speed_mode
   (&optional __fan_speed_mode)
   (if __fan_speed_mode (setq _fan_speed_mode __fan_speed_mode)) _fan_speed_mode)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _child_frame_id
    4 (length _child_frame_id)
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; geometry_msgs/Vector3 _velocity
    (send _velocity :serialization-length)
    ;; geometry_msgs/Vector3 _gyro_bias
    (send _gyro_bias :serialization-length)
    ;; geometry_msgs/Vector3 _accel_bias
    (send _accel_bias :serialization-length)
    ;; float32[15] _cov_diag
    (* 4    15)
    ;; uint8 _confidence
    1
    ;; uint32 _num_detected_of_features
    4
    ;; uint32 _num_detected_ar_features
    4
    ;; uint32 _num_detected_ml_features
    4
    ;; uint32 _iterations
    4
    ;; float32 _optimization_time
    4
    ;; float32 _update_time
    4
    ;; float32 _callbacks_time
    4
    ;; float32 _nodelet_runtime
    4
    ;; uint32 _num_factors
    4
    ;; uint32 _num_of_factors
    4
    ;; uint32 _num_ml_projection_factors
    4
    ;; uint32 _num_ml_pose_factors
    4
    ;; uint32 _num_states
    4
    ;; bool _standstill
    1
    ;; bool _estimating_bias
    1
    ;; uint8 _fan_speed_mode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _child_frame_id
       (write-long (length _child_frame_id) s) (princ _child_frame_id s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; geometry_msgs/Vector3 _velocity
       (send _velocity :serialize s)
     ;; geometry_msgs/Vector3 _gyro_bias
       (send _gyro_bias :serialize s)
     ;; geometry_msgs/Vector3 _accel_bias
       (send _accel_bias :serialize s)
     ;; float32[15] _cov_diag
     (dotimes (i 15)
       (sys::poke (elt _cov_diag i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; uint8 _confidence
       (write-byte _confidence s)
     ;; uint32 _num_detected_of_features
       (write-long _num_detected_of_features s)
     ;; uint32 _num_detected_ar_features
       (write-long _num_detected_ar_features s)
     ;; uint32 _num_detected_ml_features
       (write-long _num_detected_ml_features s)
     ;; uint32 _iterations
       (write-long _iterations s)
     ;; float32 _optimization_time
       (sys::poke _optimization_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _update_time
       (sys::poke _update_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _callbacks_time
       (sys::poke _callbacks_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _nodelet_runtime
       (sys::poke _nodelet_runtime (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _num_factors
       (write-long _num_factors s)
     ;; uint32 _num_of_factors
       (write-long _num_of_factors s)
     ;; uint32 _num_ml_projection_factors
       (write-long _num_ml_projection_factors s)
     ;; uint32 _num_ml_pose_factors
       (write-long _num_ml_pose_factors s)
     ;; uint32 _num_states
       (write-long _num_states s)
     ;; bool _standstill
       (if _standstill (write-byte -1 s) (write-byte 0 s))
     ;; bool _estimating_bias
       (if _estimating_bias (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _fan_speed_mode
       (write-byte _fan_speed_mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _child_frame_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _child_frame_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; geometry_msgs/Vector3 _velocity
     (send _velocity :deserialize buf ptr-) (incf ptr- (send _velocity :serialization-length))
   ;; geometry_msgs/Vector3 _gyro_bias
     (send _gyro_bias :deserialize buf ptr-) (incf ptr- (send _gyro_bias :serialization-length))
   ;; geometry_msgs/Vector3 _accel_bias
     (send _accel_bias :deserialize buf ptr-) (incf ptr- (send _accel_bias :serialization-length))
   ;; float32[15] _cov_diag
   (dotimes (i (length _cov_diag))
     (setf (elt _cov_diag i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; uint8 _confidence
     (setq _confidence (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint32 _num_detected_of_features
     (setq _num_detected_of_features (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _num_detected_ar_features
     (setq _num_detected_ar_features (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _num_detected_ml_features
     (setq _num_detected_ml_features (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _iterations
     (setq _iterations (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _optimization_time
     (setq _optimization_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _update_time
     (setq _update_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _callbacks_time
     (setq _callbacks_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _nodelet_runtime
     (setq _nodelet_runtime (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _num_factors
     (setq _num_factors (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _num_of_factors
     (setq _num_of_factors (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _num_ml_projection_factors
     (setq _num_ml_projection_factors (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _num_ml_pose_factors
     (setq _num_ml_pose_factors (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _num_states
     (setq _num_states (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _standstill
     (setq _standstill (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _estimating_bias
     (setq _estimating_bias (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _fan_speed_mode
     (setq _fan_speed_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get ff_msgs::GraphState :md5sum-) "d0020fbc20fe81214e0f3f2b41dd4c22")
(setf (get ff_msgs::GraphState :datatype-) "ff_msgs/GraphState")
(setf (get ff_msgs::GraphState :definition-)
      "# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the \"License\"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.
#

std_msgs/Header header # header with timestamp
string child_frame_id # frame ID
# State Estimates
geometry_msgs/Pose pose # world_T_body 
geometry_msgs/Vector3 velocity # body velocity
geometry_msgs/Vector3 gyro_bias # estimated gyro bias
geometry_msgs/Vector3 accel_bias # estimated accel bias
# Covariances/Confidences
# covariance diagonal. 1-3 orientation, 4-6 gyro bias, 7-9 velocity, 10-12 accel bias, 13-15 position
float32[15] cov_diag
# confidence in estimate. 0 is good, 1 is a bit confused, 2 is lost
uint8 confidence
uint8 CONFIDENCE_GOOD = 0	# Tracking well
uint8 CONFIDENCE_POOR = 1	# Tracking poorly
uint8 CONFIDENCE_LOST = 2	# We are lost
# Stats
uint32 num_detected_of_features  
uint32 num_detected_ar_features 
uint32 num_detected_ml_features 
uint32 iterations # Optimization iterations
float32 optimization_time
float32 update_time # Include optimization_time and other operations to add data to graph
float32 callbacks_time # Includes processing msgs and their callbacks
float32 nodelet_runtime # Total runtime of nodelet iteration.  Includes update and callback time
uint32 num_factors
uint32 num_of_factors
uint32 num_ml_projection_factors
uint32 num_ml_pose_factors
uint32 num_states
# Status
bool standstill
bool estimating_bias # Are we busy estimating the bias?
uint8 fan_speed_mode # Used for imu filtering

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :ff_msgs/GraphState "d0020fbc20fe81214e0f3f2b41dd4c22")


