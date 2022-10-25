;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::FaultData)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'FaultData (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::FAULTDATA")
  (make-package "FF_MSGS::FAULTDATA"))

(in-package "ROS")
;;//! \htmlinclude FaultData.msg.html


(intern "*DATA_TYPE_FLOAT*" (find-package "FF_MSGS::FAULTDATA"))
(shadow '*DATA_TYPE_FLOAT* (find-package "FF_MSGS::FAULTDATA"))
(defconstant ff_msgs::FaultData::*DATA_TYPE_FLOAT* 0)
(intern "*DATA_TYPE_INT*" (find-package "FF_MSGS::FAULTDATA"))
(shadow '*DATA_TYPE_INT* (find-package "FF_MSGS::FAULTDATA"))
(defconstant ff_msgs::FaultData::*DATA_TYPE_INT* 1)
(intern "*DATA_TYPE_STRING*" (find-package "FF_MSGS::FAULTDATA"))
(shadow '*DATA_TYPE_STRING* (find-package "FF_MSGS::FAULTDATA"))
(defconstant ff_msgs::FaultData::*DATA_TYPE_STRING* 2)
(defclass ff_msgs::FaultData
  :super ros::object
  :slots (_key _data_type _f _i _s ))

(defmethod ff_msgs::FaultData
  (:init
   (&key
    ((:key __key) "")
    ((:data_type __data_type) 0)
    ((:f __f) 0.0)
    ((:i __i) 0)
    ((:s __s) "")
    )
   (send-super :init)
   (setq _key (string __key))
   (setq _data_type (round __data_type))
   (setq _f (float __f))
   (setq _i (round __i))
   (setq _s (string __s))
   self)
  (:key
   (&optional __key)
   (if __key (setq _key __key)) _key)
  (:data_type
   (&optional __data_type)
   (if __data_type (setq _data_type __data_type)) _data_type)
  (:f
   (&optional __f)
   (if __f (setq _f __f)) _f)
  (:i
   (&optional __i)
   (if __i (setq _i __i)) _i)
  (:s
   (&optional __s)
   (if __s (setq _s __s)) _s)
  (:serialization-length
   ()
   (+
    ;; string _key
    4 (length _key)
    ;; uint8 _data_type
    1
    ;; float32 _f
    4
    ;; int32 _i
    4
    ;; string _s
    4 (length _s)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _key
       (write-long (length _key) s) (princ _key s)
     ;; uint8 _data_type
       (write-byte _data_type s)
     ;; float32 _f
       (sys::poke _f (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _i
       (write-long _i s)
     ;; string _s
       (write-long (length _s) s) (princ _s s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _key
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _key (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _data_type
     (setq _data_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _f
     (setq _f (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _i
     (setq _i (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _s
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _s (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get ff_msgs::FaultData :md5sum-) "632c6de83aa53364cbd36514ffa5c853")
(setf (get ff_msgs::FaultData :datatype-) "ff_msgs/FaultData")
(setf (get ff_msgs::FaultData :definition-)
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
# Fault data messsage contains information of why the fault occurred

uint8 DATA_TYPE_FLOAT   = 0   # Data in this msg is of type float
uint8 DATA_TYPE_INT     = 1   # Data in this msg is of type int
uint8 DATA_TYPE_STRING  = 2   # Data in this msg is of type string

string key  # Specifies what the data in the msg is, can only be 32 chars long

uint8 data_type   # Specifies the type of data in the message

float32 f   # Value used for fault analysis, data_type must be 0
int32 i     # Value used for fault analysis, data_type must be 1
string s    # String used for fault analysis, data_type must be 2

")



(provide :ff_msgs/FaultData "632c6de83aa53364cbd36514ffa5c853")


