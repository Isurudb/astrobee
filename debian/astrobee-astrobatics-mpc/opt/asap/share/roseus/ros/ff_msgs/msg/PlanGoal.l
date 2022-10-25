;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::PlanGoal)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'PlanGoal (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::PLANGOAL")
  (make-package "FF_MSGS::PLANGOAL"))

(in-package "ROS")
;;//! \htmlinclude PlanGoal.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass ff_msgs::PlanGoal
  :super ros::object
  :slots (_states _faceforward _check_obstacles _desired_vel _desired_accel _desired_omega _desired_alpha _desired_rate _max_time ))

(defmethod ff_msgs::PlanGoal
  (:init
   (&key
    ((:states __states) (let (r) (dotimes (i 0) (push (instance geometry_msgs::PoseStamped :init) r)) r))
    ((:faceforward __faceforward) nil)
    ((:check_obstacles __check_obstacles) nil)
    ((:desired_vel __desired_vel) 0.0)
    ((:desired_accel __desired_accel) 0.0)
    ((:desired_omega __desired_omega) 0.0)
    ((:desired_alpha __desired_alpha) 0.0)
    ((:desired_rate __desired_rate) 0.0)
    ((:max_time __max_time) (instance ros::time :init))
    )
   (send-super :init)
   (setq _states __states)
   (setq _faceforward __faceforward)
   (setq _check_obstacles __check_obstacles)
   (setq _desired_vel (float __desired_vel))
   (setq _desired_accel (float __desired_accel))
   (setq _desired_omega (float __desired_omega))
   (setq _desired_alpha (float __desired_alpha))
   (setq _desired_rate (float __desired_rate))
   (setq _max_time __max_time)
   self)
  (:states
   (&rest __states)
   (if (keywordp (car __states))
       (send* _states __states)
     (progn
       (if __states (setq _states (car __states)))
       _states)))
  (:faceforward
   (&optional __faceforward)
   (if __faceforward (setq _faceforward __faceforward)) _faceforward)
  (:check_obstacles
   (&optional __check_obstacles)
   (if __check_obstacles (setq _check_obstacles __check_obstacles)) _check_obstacles)
  (:desired_vel
   (&optional __desired_vel)
   (if __desired_vel (setq _desired_vel __desired_vel)) _desired_vel)
  (:desired_accel
   (&optional __desired_accel)
   (if __desired_accel (setq _desired_accel __desired_accel)) _desired_accel)
  (:desired_omega
   (&optional __desired_omega)
   (if __desired_omega (setq _desired_omega __desired_omega)) _desired_omega)
  (:desired_alpha
   (&optional __desired_alpha)
   (if __desired_alpha (setq _desired_alpha __desired_alpha)) _desired_alpha)
  (:desired_rate
   (&optional __desired_rate)
   (if __desired_rate (setq _desired_rate __desired_rate)) _desired_rate)
  (:max_time
   (&optional __max_time)
   (if __max_time (setq _max_time __max_time)) _max_time)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PoseStamped[] _states
    (apply #'+ (send-all _states :serialization-length)) 4
    ;; bool _faceforward
    1
    ;; bool _check_obstacles
    1
    ;; float32 _desired_vel
    4
    ;; float32 _desired_accel
    4
    ;; float32 _desired_omega
    4
    ;; float32 _desired_alpha
    4
    ;; float32 _desired_rate
    4
    ;; duration _max_time
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PoseStamped[] _states
     (write-long (length _states) s)
     (dolist (elem _states)
       (send elem :serialize s)
       )
     ;; bool _faceforward
       (if _faceforward (write-byte -1 s) (write-byte 0 s))
     ;; bool _check_obstacles
       (if _check_obstacles (write-byte -1 s) (write-byte 0 s))
     ;; float32 _desired_vel
       (sys::poke _desired_vel (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _desired_accel
       (sys::poke _desired_accel (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _desired_omega
       (sys::poke _desired_omega (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _desired_alpha
       (sys::poke _desired_alpha (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _desired_rate
       (sys::poke _desired_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; duration _max_time
       (write-long (send _max_time :sec) s) (write-long (send _max_time :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PoseStamped[] _states
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _states (let (r) (dotimes (i n) (push (instance geometry_msgs::PoseStamped :init) r)) r))
     (dolist (elem- _states)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; bool _faceforward
     (setq _faceforward (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _check_obstacles
     (setq _check_obstacles (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _desired_vel
     (setq _desired_vel (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _desired_accel
     (setq _desired_accel (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _desired_omega
     (setq _desired_omega (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _desired_alpha
     (setq _desired_alpha (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _desired_rate
     (setq _desired_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; duration _max_time
     (send _max_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _max_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ff_msgs::PlanGoal :md5sum-) "8a3c023dc4c031730a0eb5ee3812c31e")
(setf (get ff_msgs::PlanGoal :datatype-) "ff_msgs/PlanGoal")
(setf (get ff_msgs::PlanGoal :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
# Copyright (c) 2017, United States Government, as represented by the
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
# This message describes the PLAN action offered by the PLANNER

geometry_msgs/PoseStamped[] states            # Desired state sequence

bool faceforward                              # Face-forward trajectory?
bool check_obstacles                          # Check against obstacles?

float32 desired_vel                           # Desired (max) velocity
float32 desired_accel                         # Desired (max) accel
float32 desired_omega                         # Desired (max) omega
float32 desired_alpha                         # Desired (max) alpha
float32 desired_rate                          # Desired rate

duration max_time                             # Max generation time


================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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

")



(provide :ff_msgs/PlanGoal "8a3c023dc4c031730a0eb5ee3812c31e")


