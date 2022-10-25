;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::GetMap)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'GetMap (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::GETMAP")
  (make-package "FF_MSGS::GETMAP"))
(unless (find-package "FF_MSGS::GETMAPREQUEST")
  (make-package "FF_MSGS::GETMAPREQUEST"))
(unless (find-package "FF_MSGS::GETMAPRESPONSE")
  (make-package "FF_MSGS::GETMAPRESPONSE"))

(in-package "ROS")



(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass ff_msgs::GetMapRequest
  :super ros::object
  :slots ())

(defmethod ff_msgs::GetMapRequest
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

(defclass ff_msgs::GetMapResponse
  :super ros::object
  :slots (_points _resolution _free ))

(defmethod ff_msgs::GetMapResponse
  (:init
   (&key
    ((:points __points) (instance sensor_msgs::PointCloud2 :init))
    ((:resolution __resolution) 0.0)
    ((:free __free) nil)
    )
   (send-super :init)
   (setq _points __points)
   (setq _resolution (float __resolution))
   (setq _free __free)
   self)
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:resolution
   (&optional __resolution)
   (if __resolution (setq _resolution __resolution)) _resolution)
  (:free
   (&optional __free)
   (if __free (setq _free __free)) _free)
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/PointCloud2 _points
    (send _points :serialization-length)
    ;; float32 _resolution
    4
    ;; bool _free
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/PointCloud2 _points
       (send _points :serialize s)
     ;; float32 _resolution
       (sys::poke _resolution (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _free
       (if _free (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/PointCloud2 _points
     (send _points :deserialize buf ptr-) (incf ptr- (send _points :serialization-length))
   ;; float32 _resolution
     (setq _resolution (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _free
     (setq _free (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ff_msgs::GetMap
  :super ros::object
  :slots ())

(setf (get ff_msgs::GetMap :md5sum-) "2fabf4f6566443e23e62f744b9ba2ad6")
(setf (get ff_msgs::GetMap :datatype-) "ff_msgs/GetMap")
(setf (get ff_msgs::GetMap :request) ff_msgs::GetMapRequest)
(setf (get ff_msgs::GetMap :response) ff_msgs::GetMapResponse)

(defmethod ff_msgs::GetMapRequest
  (:response () (instance ff_msgs::GetMapResponse :init)))

(setf (get ff_msgs::GetMapRequest :md5sum-) "2fabf4f6566443e23e62f744b9ba2ad6")
(setf (get ff_msgs::GetMapRequest :datatype-) "ff_msgs/GetMapRequest")
(setf (get ff_msgs::GetMapRequest :definition-)
      "


















---
sensor_msgs/PointCloud2 points
float32 resolution
bool free

================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the \"fields\" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

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
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field
")

(setf (get ff_msgs::GetMapResponse :md5sum-) "2fabf4f6566443e23e62f744b9ba2ad6")
(setf (get ff_msgs::GetMapResponse :datatype-) "ff_msgs/GetMapResponse")
(setf (get ff_msgs::GetMapResponse :definition-)
      "


















---
sensor_msgs/PointCloud2 points
float32 resolution
bool free

================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the \"fields\" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

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
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field
")



(provide :ff_msgs/GetMap "2fabf4f6566443e23e62f744b9ba2ad6")


