;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::Zone)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'Zone (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::ZONE")
  (make-package "FF_MSGS::ZONE"))

(in-package "ROS")
;;//! \htmlinclude Zone.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(intern "*KEEPOUT*" (find-package "FF_MSGS::ZONE"))
(shadow '*KEEPOUT* (find-package "FF_MSGS::ZONE"))
(defconstant ff_msgs::Zone::*KEEPOUT* 0)
(intern "*KEEPIN*" (find-package "FF_MSGS::ZONE"))
(shadow '*KEEPIN* (find-package "FF_MSGS::ZONE"))
(defconstant ff_msgs::Zone::*KEEPIN* 1)
(intern "*CLUTTER*" (find-package "FF_MSGS::ZONE"))
(shadow '*CLUTTER* (find-package "FF_MSGS::ZONE"))
(defconstant ff_msgs::Zone::*CLUTTER* 2)
(defclass ff_msgs::Zone
  :super ros::object
  :slots (_name _index _type _min _max ))

(defmethod ff_msgs::Zone
  (:init
   (&key
    ((:name __name) "")
    ((:index __index) 0)
    ((:type __type) 0)
    ((:min __min) (instance geometry_msgs::Vector3 :init))
    ((:max __max) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _index (round __index))
   (setq _type (round __type))
   (setq _min __min)
   (setq _max __max)
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:min
   (&rest __min)
   (if (keywordp (car __min))
       (send* _min __min)
     (progn
       (if __min (setq _min (car __min)))
       _min)))
  (:max
   (&rest __max)
   (if (keywordp (car __max))
       (send* _max __max)
     (progn
       (if __max (setq _max (car __max)))
       _max)))
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; int32 _index
    4
    ;; uint8 _type
    1
    ;; geometry_msgs/Vector3 _min
    (send _min :serialization-length)
    ;; geometry_msgs/Vector3 _max
    (send _max :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; int32 _index
       (write-long _index s)
     ;; uint8 _type
       (write-byte _type s)
     ;; geometry_msgs/Vector3 _min
       (send _min :serialize s)
     ;; geometry_msgs/Vector3 _max
       (send _max :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _index
     (setq _index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _type
     (setq _type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; geometry_msgs/Vector3 _min
     (send _min :deserialize buf ptr-) (incf ptr- (send _min :serialization-length))
   ;; geometry_msgs/Vector3 _max
     (send _max :deserialize buf ptr-) (incf ptr- (send _max :serialization-length))
   ;;
   self)
  )

(setf (get ff_msgs::Zone :md5sum-) "eb4a7bedb72c164486d2ac45ba0a7b8e")
(setf (get ff_msgs::Zone :datatype-) "ff_msgs/Zone")
(setf (get ff_msgs::Zone :definition-)
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
# This message defines a zone, such as a Keepin or Keeoput.

string name                   # Name of zone

# A name can refer to multiple zones. This is the index of the zone with respect
# to the zone name
int32 index

# Zone type
uint8 KEEPOUT = 0       # An area the freeflyer should stay out of
uint8 KEEPIN  = 1       # An area the freeflyer can fly freely in 
uint8 CLUTTER = 2       # An area that the freeflyer should avoid due to clutter

uint8 type              # Whether the zone is a keepin, keepout, or clutter

geometry_msgs/Vector3 min   # One corner of the zone
geometry_msgs/Vector3 max   # The opposite corner of the zone

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



(provide :ff_msgs/Zone "eb4a7bedb72c164486d2ac45ba0a7b8e")


