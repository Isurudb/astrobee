;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::CompressedFile)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'CompressedFile (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::COMPRESSEDFILE")
  (make-package "FF_MSGS::COMPRESSEDFILE"))

(in-package "ROS")
;;//! \htmlinclude CompressedFile.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*TYPE_NONE*" (find-package "FF_MSGS::COMPRESSEDFILE"))
(shadow '*TYPE_NONE* (find-package "FF_MSGS::COMPRESSEDFILE"))
(defconstant ff_msgs::CompressedFile::*TYPE_NONE* 0)
(intern "*TYPE_DEFLATE*" (find-package "FF_MSGS::COMPRESSEDFILE"))
(shadow '*TYPE_DEFLATE* (find-package "FF_MSGS::COMPRESSEDFILE"))
(defconstant ff_msgs::CompressedFile::*TYPE_DEFLATE* 1)
(intern "*TYPE_BZ2*" (find-package "FF_MSGS::COMPRESSEDFILE"))
(shadow '*TYPE_BZ2* (find-package "FF_MSGS::COMPRESSEDFILE"))
(defconstant ff_msgs::CompressedFile::*TYPE_BZ2* 2)
(intern "*TYPE_GZ*" (find-package "FF_MSGS::COMPRESSEDFILE"))
(shadow '*TYPE_GZ* (find-package "FF_MSGS::COMPRESSEDFILE"))
(defconstant ff_msgs::CompressedFile::*TYPE_GZ* 3)
(intern "*TYPE_ZIP*" (find-package "FF_MSGS::COMPRESSEDFILE"))
(shadow '*TYPE_ZIP* (find-package "FF_MSGS::COMPRESSEDFILE"))
(defconstant ff_msgs::CompressedFile::*TYPE_ZIP* 4)
(defclass ff_msgs::CompressedFile
  :super ros::object
  :slots (_header _id _type _file ))

(defmethod ff_msgs::CompressedFile
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:id __id) 0)
    ((:type __type) 0)
    ((:file __file) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _header __header)
   (setq _id (round __id))
   (setq _type (round __type))
   (setq _file __file)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:file
   (&optional __file)
   (if __file (setq _file __file)) _file)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _id
    4
    ;; uint8 _type
    1
    ;; uint8[] _file
    (* 1    (length _file)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _id
       (write-long _id s)
     ;; uint8 _type
       (write-byte _type s)
     ;; uint8[] _file
     (write-long (length _file) s)
     (princ _file s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _type
     (setq _type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8[] _file
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _file (make-array n :element-type :char))
     (replace _file buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get ff_msgs::CompressedFile :md5sum-) "2566df54a1dc0c50a1ff95f06b4382df")
(setf (get ff_msgs::CompressedFile :datatype-) "ff_msgs/CompressedFile")
(setf (get ff_msgs::CompressedFile :definition-)
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
# A compressed file that represents a CompressedFile from
# DDS. Used to send a compressed files to the executive.
# DDS is constrained to a 128k chunk of data, we are only
# limited by our imaginations (and the size of an unsigned
# 32-bit integer)

# Header 
std_msgs/Header header

# Unique file identification
int32 id

uint8 TYPE_NONE = 0
uint8 TYPE_DEFLATE = 1
uint8 TYPE_BZ2 = 2
uint8 TYPE_GZ = 3
uint8 TYPE_ZIP = 4

# Type of compression
uint8 type

# File contents
uint8[] file

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

")



(provide :ff_msgs/CompressedFile "2566df54a1dc0c50a1ff95f06b4382df")


