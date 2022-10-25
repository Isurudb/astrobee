;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::SetInertia)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'SetInertia (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::SETINERTIA")
  (make-package "FF_MSGS::SETINERTIA"))
(unless (find-package "FF_MSGS::SETINERTIAREQUEST")
  (make-package "FF_MSGS::SETINERTIAREQUEST"))
(unless (find-package "FF_MSGS::SETINERTIARESPONSE")
  (make-package "FF_MSGS::SETINERTIARESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass ff_msgs::SetInertiaRequest
  :super ros::object
  :slots (_inertia ))

(defmethod ff_msgs::SetInertiaRequest
  (:init
   (&key
    ((:inertia __inertia) (instance geometry_msgs::InertiaStamped :init))
    )
   (send-super :init)
   (setq _inertia __inertia)
   self)
  (:inertia
   (&rest __inertia)
   (if (keywordp (car __inertia))
       (send* _inertia __inertia)
     (progn
       (if __inertia (setq _inertia (car __inertia)))
       _inertia)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/InertiaStamped _inertia
    (send _inertia :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/InertiaStamped _inertia
       (send _inertia :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/InertiaStamped _inertia
     (send _inertia :deserialize buf ptr-) (incf ptr- (send _inertia :serialization-length))
   ;;
   self)
  )

(defclass ff_msgs::SetInertiaResponse
  :super ros::object
  :slots (_success ))

(defmethod ff_msgs::SetInertiaResponse
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

(defclass ff_msgs::SetInertia
  :super ros::object
  :slots ())

(setf (get ff_msgs::SetInertia :md5sum-) "bed413bb59ac0b22ecff0ffcb6e7dcc2")
(setf (get ff_msgs::SetInertia :datatype-) "ff_msgs/SetInertia")
(setf (get ff_msgs::SetInertia :request) ff_msgs::SetInertiaRequest)
(setf (get ff_msgs::SetInertia :response) ff_msgs::SetInertiaResponse)

(defmethod ff_msgs::SetInertiaRequest
  (:response () (instance ff_msgs::SetInertiaResponse :init)))

(setf (get ff_msgs::SetInertiaRequest :md5sum-) "bed413bb59ac0b22ecff0ffcb6e7dcc2")
(setf (get ff_msgs::SetInertiaRequest :datatype-) "ff_msgs/SetInertiaRequest")
(setf (get ff_msgs::SetInertiaRequest :definition-)
      "


















geometry_msgs/InertiaStamped inertia

================================================================================
MSG: geometry_msgs/InertiaStamped
Header header
Inertia inertia

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
MSG: geometry_msgs/Inertia
# Mass [kg]
float64 m

# Center of mass [m]
geometry_msgs/Vector3 com

# Inertia Tensor [kg-m^2]
#     | ixx ixy ixz |
# I = | ixy iyy iyz |
#     | ixz iyz izz |
float64 ixx
float64 ixy
float64 ixz
float64 iyy
float64 iyz
float64 izz

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

(setf (get ff_msgs::SetInertiaResponse :md5sum-) "bed413bb59ac0b22ecff0ffcb6e7dcc2")
(setf (get ff_msgs::SetInertiaResponse :datatype-) "ff_msgs/SetInertiaResponse")
(setf (get ff_msgs::SetInertiaResponse :definition-)
      "


















geometry_msgs/InertiaStamped inertia

================================================================================
MSG: geometry_msgs/InertiaStamped
Header header
Inertia inertia

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
MSG: geometry_msgs/Inertia
# Mass [kg]
float64 m

# Center of mass [m]
geometry_msgs/Vector3 com

# Inertia Tensor [kg-m^2]
#     | ixx ixy ixz |
# I = | ixy iyy iyz |
#     | ixz iyz izz |
float64 ixx
float64 ixy
float64 ixz
float64 iyy
float64 iyz
float64 izz

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



(provide :ff_msgs/SetInertia "bed413bb59ac0b22ecff0ffcb6e7dcc2")


