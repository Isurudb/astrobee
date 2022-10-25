;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::PicoflexxIntermediateData)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'PicoflexxIntermediateData (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::PICOFLEXXINTERMEDIATEDATA")
  (make-package "FF_MSGS::PICOFLEXXINTERMEDIATEDATA"))

(in-package "ROS")
;;//! \htmlinclude PicoflexxIntermediateData.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::PicoflexxIntermediateData
  :super ros::object
  :slots (_header _frequency _exposure _raw ))

(defmethod ff_msgs::PicoflexxIntermediateData
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:frequency __frequency) (make-array 0 :initial-element 0 :element-type :integer))
    ((:exposure __exposure) (make-array 0 :initial-element 0 :element-type :integer))
    ((:raw __raw) (instance sensor_msgs::Image :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _frequency __frequency)
   (setq _exposure __exposure)
   (setq _raw __raw)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:frequency
   (&optional __frequency)
   (if __frequency (setq _frequency __frequency)) _frequency)
  (:exposure
   (&optional __exposure)
   (if __exposure (setq _exposure __exposure)) _exposure)
  (:raw
   (&rest __raw)
   (if (keywordp (car __raw))
       (send* _raw __raw)
     (progn
       (if __raw (setq _raw (car __raw)))
       _raw)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32[] _frequency
    (* 4    (length _frequency)) 4
    ;; uint32[] _exposure
    (* 4    (length _exposure)) 4
    ;; sensor_msgs/Image _raw
    (send _raw :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32[] _frequency
     (write-long (length _frequency) s)
     (dotimes (i (length _frequency))
       (write-long (elt _frequency i) s)
       )
     ;; uint32[] _exposure
     (write-long (length _exposure) s)
     (dotimes (i (length _exposure))
       (write-long (elt _exposure i) s)
       )
     ;; sensor_msgs/Image _raw
       (send _raw :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32[] _frequency
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _frequency (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _frequency i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; uint32[] _exposure
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _exposure (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _exposure i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; sensor_msgs/Image _raw
     (send _raw :deserialize buf ptr-) (incf ptr- (send _raw :serialization-length))
   ;;
   self)
  )

(setf (get ff_msgs::PicoflexxIntermediateData :md5sum-) "831d0b822464699c0a04d899b4867483")
(setf (get ff_msgs::PicoflexxIntermediateData :datatype-) "ff_msgs/PicoflexxIntermediateData")
(setf (get ff_msgs::PicoflexxIntermediateData :definition-)
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
# under the License.      #
#
# Message for encoding Picoflexx imtermediate data 

std_msgs/Header header
uint32[] frequency
uint32[] exposure
sensor_msgs/Image raw
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
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of cameara
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

")



(provide :ff_msgs/PicoflexxIntermediateData "831d0b822464699c0a04d899b4867483")


