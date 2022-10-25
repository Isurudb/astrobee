;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::SetZones)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'SetZones (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::SETZONES")
  (make-package "FF_MSGS::SETZONES"))
(unless (find-package "FF_MSGS::SETZONESREQUEST")
  (make-package "FF_MSGS::SETZONESREQUEST"))
(unless (find-package "FF_MSGS::SETZONESRESPONSE")
  (make-package "FF_MSGS::SETZONESRESPONSE"))

(in-package "ROS")





(defclass ff_msgs::SetZonesRequest
  :super ros::object
  :slots (_timestamp _zones ))

(defmethod ff_msgs::SetZonesRequest
  (:init
   (&key
    ((:timestamp __timestamp) (instance ros::time :init))
    ((:zones __zones) (let (r) (dotimes (i 0) (push (instance ff_msgs::Zone :init) r)) r))
    )
   (send-super :init)
   (setq _timestamp __timestamp)
   (setq _zones __zones)
   self)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:zones
   (&rest __zones)
   (if (keywordp (car __zones))
       (send* _zones __zones)
     (progn
       (if __zones (setq _zones (car __zones)))
       _zones)))
  (:serialization-length
   ()
   (+
    ;; time _timestamp
    8
    ;; ff_msgs/Zone[] _zones
    (apply #'+ (send-all _zones :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _timestamp
       (write-long (send _timestamp :sec) s) (write-long (send _timestamp :nsec) s)
     ;; ff_msgs/Zone[] _zones
     (write-long (length _zones) s)
     (dolist (elem _zones)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _timestamp
     (send _timestamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _timestamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; ff_msgs/Zone[] _zones
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _zones (let (r) (dotimes (i n) (push (instance ff_msgs::Zone :init) r)) r))
     (dolist (elem- _zones)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass ff_msgs::SetZonesResponse
  :super ros::object
  :slots (_success ))

(defmethod ff_msgs::SetZonesResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ff_msgs::SetZones
  :super ros::object
  :slots ())

(setf (get ff_msgs::SetZones :md5sum-) "409154f0ce409359cbde795e43243c62")
(setf (get ff_msgs::SetZones :datatype-) "ff_msgs/SetZones")
(setf (get ff_msgs::SetZones :request) ff_msgs::SetZonesRequest)
(setf (get ff_msgs::SetZones :response) ff_msgs::SetZonesResponse)

(defmethod ff_msgs::SetZonesRequest
  (:response () (instance ff_msgs::SetZonesResponse :init)))

(setf (get ff_msgs::SetZonesRequest :md5sum-) "409154f0ce409359cbde795e43243c62")
(setf (get ff_msgs::SetZonesRequest :datatype-) "ff_msgs/SetZonesRequest")
(setf (get ff_msgs::SetZonesRequest :definition-)
      "


















time timestamp
ff_msgs/Zone[] zones

================================================================================
MSG: ff_msgs/Zone
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
---
bool success
")

(setf (get ff_msgs::SetZonesResponse :md5sum-) "409154f0ce409359cbde795e43243c62")
(setf (get ff_msgs::SetZonesResponse :datatype-) "ff_msgs/SetZonesResponse")
(setf (get ff_msgs::SetZonesResponse :definition-)
      "


















time timestamp
ff_msgs/Zone[] zones

================================================================================
MSG: ff_msgs/Zone
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
---
bool success
")



(provide :ff_msgs/SetZones "409154f0ce409359cbde795e43243c62")


