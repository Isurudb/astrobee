;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::GetOccupancyMap)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'GetOccupancyMap (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::GETOCCUPANCYMAP")
  (make-package "FF_MSGS::GETOCCUPANCYMAP"))
(unless (find-package "FF_MSGS::GETOCCUPANCYMAPREQUEST")
  (make-package "FF_MSGS::GETOCCUPANCYMAPREQUEST"))
(unless (find-package "FF_MSGS::GETOCCUPANCYMAPRESPONSE")
  (make-package "FF_MSGS::GETOCCUPANCYMAPRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass ff_msgs::GetOccupancyMapRequest
  :super ros::object
  :slots ())

(defmethod ff_msgs::GetOccupancyMapRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass ff_msgs::GetOccupancyMapResponse
  :super ros::object
  :slots (_timestamp _map _origin _dim _resolution ))

(defmethod ff_msgs::GetOccupancyMapResponse
  (:init
   (&key
    ((:timestamp __timestamp) (instance ros::time :init))
    ((:map __map) (make-array 0 :initial-element 0 :element-type :integer))
    ((:origin __origin) (instance geometry_msgs::Vector3 :init))
    ((:dim __dim) (instance geometry_msgs::Vector3 :init))
    ((:resolution __resolution) 0.0)
    )
   (send-super :init)
   (setq _timestamp __timestamp)
   (setq _map __map)
   (setq _origin __origin)
   (setq _dim __dim)
   (setq _resolution (float __resolution))
   self)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:map
   (&optional __map)
   (if __map (setq _map __map)) _map)
  (:origin
   (&rest __origin)
   (if (keywordp (car __origin))
       (send* _origin __origin)
     (progn
       (if __origin (setq _origin (car __origin)))
       _origin)))
  (:dim
   (&rest __dim)
   (if (keywordp (car __dim))
       (send* _dim __dim)
     (progn
       (if __dim (setq _dim (car __dim)))
       _dim)))
  (:resolution
   (&optional __resolution)
   (if __resolution (setq _resolution __resolution)) _resolution)
  (:serialization-length
   ()
   (+
    ;; time _timestamp
    8
    ;; int8[] _map
    (* 1    (length _map)) 4
    ;; geometry_msgs/Vector3 _origin
    (send _origin :serialization-length)
    ;; geometry_msgs/Vector3 _dim
    (send _dim :serialization-length)
    ;; float32 _resolution
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _timestamp
       (write-long (send _timestamp :sec) s) (write-long (send _timestamp :nsec) s)
     ;; int8[] _map
     (write-long (length _map) s)
     (dotimes (i (length _map))
       (write-byte (elt _map i) s)
       )
     ;; geometry_msgs/Vector3 _origin
       (send _origin :serialize s)
     ;; geometry_msgs/Vector3 _dim
       (send _dim :serialize s)
     ;; float32 _resolution
       (sys::poke _resolution (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _timestamp
     (send _timestamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _timestamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int8[] _map
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _map (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _map i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _map i) 127) (setf (elt _map i) (- (elt _map i) 256)))
     ))
   ;; geometry_msgs/Vector3 _origin
     (send _origin :deserialize buf ptr-) (incf ptr- (send _origin :serialization-length))
   ;; geometry_msgs/Vector3 _dim
     (send _dim :deserialize buf ptr-) (incf ptr- (send _dim :serialization-length))
   ;; float32 _resolution
     (setq _resolution (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass ff_msgs::GetOccupancyMap
  :super ros::object
  :slots ())

(setf (get ff_msgs::GetOccupancyMap :md5sum-) "7424633971d7f4a5e61061b696c2c185")
(setf (get ff_msgs::GetOccupancyMap :datatype-) "ff_msgs/GetOccupancyMap")
(setf (get ff_msgs::GetOccupancyMap :request) ff_msgs::GetOccupancyMapRequest)
(setf (get ff_msgs::GetOccupancyMap :response) ff_msgs::GetOccupancyMapResponse)

(defmethod ff_msgs::GetOccupancyMapRequest
  (:response () (instance ff_msgs::GetOccupancyMapResponse :init)))

(setf (get ff_msgs::GetOccupancyMapRequest :md5sum-) "7424633971d7f4a5e61061b696c2c185")
(setf (get ff_msgs::GetOccupancyMapRequest :datatype-) "ff_msgs/GetOccupancyMapRequest")
(setf (get ff_msgs::GetOccupancyMapRequest :definition-)
      "


















---
time timestamp

int8[] map
geometry_msgs/Vector3 origin
geometry_msgs/Vector3 dim
float32 resolution


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
")

(setf (get ff_msgs::GetOccupancyMapResponse :md5sum-) "7424633971d7f4a5e61061b696c2c185")
(setf (get ff_msgs::GetOccupancyMapResponse :datatype-) "ff_msgs/GetOccupancyMapResponse")
(setf (get ff_msgs::GetOccupancyMapResponse :definition-)
      "


















---
time timestamp

int8[] map
geometry_msgs/Vector3 origin
geometry_msgs/Vector3 dim
float32 resolution


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
")



(provide :ff_msgs/GetOccupancyMap "7424633971d7f4a5e61061b696c2c185")


