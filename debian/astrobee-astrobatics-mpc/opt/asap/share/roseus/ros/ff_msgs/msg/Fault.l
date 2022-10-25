;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::Fault)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'Fault (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::FAULT")
  (make-package "FF_MSGS::FAULT"))

(in-package "ROS")
;;//! \htmlinclude Fault.msg.html


(defclass ff_msgs::Fault
  :super ros::object
  :slots (_time_of_fault _id _msg _data ))

(defmethod ff_msgs::Fault
  (:init
   (&key
    ((:time_of_fault __time_of_fault) (instance ros::time :init))
    ((:id __id) 0)
    ((:msg __msg) "")
    ((:data __data) (let (r) (dotimes (i 0) (push (instance ff_msgs::FaultData :init) r)) r))
    )
   (send-super :init)
   (setq _time_of_fault __time_of_fault)
   (setq _id (round __id))
   (setq _msg (string __msg))
   (setq _data __data)
   self)
  (:time_of_fault
   (&optional __time_of_fault)
   (if __time_of_fault (setq _time_of_fault __time_of_fault)) _time_of_fault)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:msg
   (&optional __msg)
   (if __msg (setq _msg __msg)) _msg)
  (:data
   (&rest __data)
   (if (keywordp (car __data))
       (send* _data __data)
     (progn
       (if __data (setq _data (car __data)))
       _data)))
  (:serialization-length
   ()
   (+
    ;; time _time_of_fault
    8
    ;; uint32 _id
    4
    ;; string _msg
    4 (length _msg)
    ;; ff_msgs/FaultData[] _data
    (apply #'+ (send-all _data :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _time_of_fault
       (write-long (send _time_of_fault :sec) s) (write-long (send _time_of_fault :nsec) s)
     ;; uint32 _id
       (write-long _id s)
     ;; string _msg
       (write-long (length _msg) s) (princ _msg s)
     ;; ff_msgs/FaultData[] _data
     (write-long (length _data) s)
     (dolist (elem _data)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _time_of_fault
     (send _time_of_fault :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _time_of_fault :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _msg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _msg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; ff_msgs/FaultData[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (let (r) (dotimes (i n) (push (instance ff_msgs::FaultData :init) r)) r))
     (dolist (elem- _data)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ff_msgs::Fault :md5sum-) "67f951d3568a6651a818ff487dcc2650")
(setf (get ff_msgs::Fault :datatype-) "ff_msgs/Fault")
(setf (get ff_msgs::Fault :definition-)
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
# Fault message is used to provide all the information about an occurring fault

time time_of_fault        # Time when fault occurred

uint32 id                 # id specifying fault

string msg                # string specifying why the fault occurred

ff_msgs/FaultData[] data  # Data used for fault analysis

================================================================================
MSG: ff_msgs/FaultData
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



(provide :ff_msgs/Fault "67f951d3568a6651a818ff487dcc2650")


