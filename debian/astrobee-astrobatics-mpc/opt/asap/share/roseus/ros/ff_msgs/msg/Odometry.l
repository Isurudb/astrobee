;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::Odometry)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'Odometry (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::ODOMETRY")
  (make-package "FF_MSGS::ODOMETRY"))

(in-package "ROS")
;;//! \htmlinclude Odometry.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass ff_msgs::Odometry
  :super ros::object
  :slots (_source_time _target_time _sensor_F_source_T_target _body_F_source_T_target ))

(defmethod ff_msgs::Odometry
  (:init
   (&key
    ((:source_time __source_time) (instance ros::time :init))
    ((:target_time __target_time) (instance ros::time :init))
    ((:sensor_F_source_T_target __sensor_F_source_T_target) (instance geometry_msgs::PoseWithCovariance :init))
    ((:body_F_source_T_target __body_F_source_T_target) (instance geometry_msgs::PoseWithCovariance :init))
    )
   (send-super :init)
   (setq _source_time __source_time)
   (setq _target_time __target_time)
   (setq _sensor_F_source_T_target __sensor_F_source_T_target)
   (setq _body_F_source_T_target __body_F_source_T_target)
   self)
  (:source_time
   (&optional __source_time)
   (if __source_time (setq _source_time __source_time)) _source_time)
  (:target_time
   (&optional __target_time)
   (if __target_time (setq _target_time __target_time)) _target_time)
  (:sensor_F_source_T_target
   (&rest __sensor_F_source_T_target)
   (if (keywordp (car __sensor_F_source_T_target))
       (send* _sensor_F_source_T_target __sensor_F_source_T_target)
     (progn
       (if __sensor_F_source_T_target (setq _sensor_F_source_T_target (car __sensor_F_source_T_target)))
       _sensor_F_source_T_target)))
  (:body_F_source_T_target
   (&rest __body_F_source_T_target)
   (if (keywordp (car __body_F_source_T_target))
       (send* _body_F_source_T_target __body_F_source_T_target)
     (progn
       (if __body_F_source_T_target (setq _body_F_source_T_target (car __body_F_source_T_target)))
       _body_F_source_T_target)))
  (:serialization-length
   ()
   (+
    ;; time _source_time
    8
    ;; time _target_time
    8
    ;; geometry_msgs/PoseWithCovariance _sensor_F_source_T_target
    (send _sensor_F_source_T_target :serialization-length)
    ;; geometry_msgs/PoseWithCovariance _body_F_source_T_target
    (send _body_F_source_T_target :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _source_time
       (write-long (send _source_time :sec) s) (write-long (send _source_time :nsec) s)
     ;; time _target_time
       (write-long (send _target_time :sec) s) (write-long (send _target_time :nsec) s)
     ;; geometry_msgs/PoseWithCovariance _sensor_F_source_T_target
       (send _sensor_F_source_T_target :serialize s)
     ;; geometry_msgs/PoseWithCovariance _body_F_source_T_target
       (send _body_F_source_T_target :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _source_time
     (send _source_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _source_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; time _target_time
     (send _target_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _target_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/PoseWithCovariance _sensor_F_source_T_target
     (send _sensor_F_source_T_target :deserialize buf ptr-) (incf ptr- (send _sensor_F_source_T_target :serialization-length))
   ;; geometry_msgs/PoseWithCovariance _body_F_source_T_target
     (send _body_F_source_T_target :deserialize buf ptr-) (incf ptr- (send _body_F_source_T_target :serialization-length))
   ;;
   self)
  )

(setf (get ff_msgs::Odometry :md5sum-) "422b88b6dc476361c3b1485e5b6113f4")
(setf (get ff_msgs::Odometry :datatype-) "ff_msgs/Odometry")
(setf (get ff_msgs::Odometry :definition-)
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

time source_time 
time target_time
geometry_msgs/PoseWithCovariance sensor_F_source_T_target
geometry_msgs/PoseWithCovariance body_F_source_T_target

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

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



(provide :ff_msgs/Odometry "422b88b6dc476361c3b1485e5b6113f4")


