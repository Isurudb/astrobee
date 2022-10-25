;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::GuestScienceState)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'GuestScienceState (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::GUESTSCIENCESTATE")
  (make-package "FF_MSGS::GUESTSCIENCESTATE"))

(in-package "ROS")
;;//! \htmlinclude GuestScienceState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::GuestScienceState
  :super ros::object
  :slots (_header _serial _runningApks ))

(defmethod ff_msgs::GuestScienceState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:serial __serial) 0)
    ((:runningApks __runningApks) (let (r) (dotimes (i 0) (push nil r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _serial (round __serial))
   (setq _runningApks __runningApks)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:serial
   (&optional __serial)
   (if __serial (setq _serial __serial)) _serial)
  (:runningApks
   (&optional __runningApks)
   (if __runningApks (setq _runningApks __runningApks)) _runningApks)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int64 _serial
    8
    ;; bool[] _runningApks
    (* 1    (length _runningApks)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int64 _serial
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _serial (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _serial) (= (length (_serial . bv)) 2)) ;; bignum
              (write-long (ash (elt (_serial . bv) 0) 0) s)
              (write-long (ash (elt (_serial . bv) 1) -1) s))
             ((and (class _serial) (= (length (_serial . bv)) 1)) ;; big1
              (write-long (elt (_serial . bv) 0) s)
              (write-long (if (>= _serial 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _serial s)(write-long (if (>= _serial 0) 0 #xffffffff) s)))
     ;; bool[] _runningApks
     (write-long (length _runningApks) s)
     (dotimes (i (length _runningApks))
       (if (elt _runningApks i) (write-byte -1 s) (write-byte 0 s))
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int64 _serial
#+(or :alpha :irix6 :x86_64)
      (setf _serial (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _serial (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; bool[] _runningApks
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _runningApks (make-list n))
     (dotimes (i n)
     (setf (elt _runningApks i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;;
   self)
  )

(setf (get ff_msgs::GuestScienceState :md5sum-) "24e5a1ba8527257cc710720a32de6e51")
(setf (get ff_msgs::GuestScienceState :datatype-) "ff_msgs/GuestScienceState")
(setf (get ff_msgs::GuestScienceState :definition-)
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
# Message used to tell which guest science apks are running

# Header with timestamp
std_msgs/Header header

# Used for guest science config and state message synchronization on the ground
int64 serial

# Tells which apks are running. Used in conjection with the guest science config
bool[] runningApks

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



(provide :ff_msgs/GuestScienceState "24e5a1ba8527257cc710720a32de6e51")


