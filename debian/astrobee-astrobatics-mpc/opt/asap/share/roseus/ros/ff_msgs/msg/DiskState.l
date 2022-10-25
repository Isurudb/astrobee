;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::DiskState)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'DiskState (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::DISKSTATE")
  (make-package "FF_MSGS::DISKSTATE"))

(in-package "ROS")
;;//! \htmlinclude DiskState.msg.html


(defclass ff_msgs::DiskState
  :super ros::object
  :slots (_path _capacity _used ))

(defmethod ff_msgs::DiskState
  (:init
   (&key
    ((:path __path) "")
    ((:capacity __capacity) 0)
    ((:used __used) 0)
    )
   (send-super :init)
   (setq _path (string __path))
   (setq _capacity (round __capacity))
   (setq _used (round __used))
   self)
  (:path
   (&optional __path)
   (if __path (setq _path __path)) _path)
  (:capacity
   (&optional __capacity)
   (if __capacity (setq _capacity __capacity)) _capacity)
  (:used
   (&optional __used)
   (if __used (setq _used __used)) _used)
  (:serialization-length
   ()
   (+
    ;; string _path
    4 (length _path)
    ;; uint64 _capacity
    8
    ;; uint64 _used
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _path
       (write-long (length _path) s) (princ _path s)
     ;; uint64 _capacity
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _capacity (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _capacity) (= (length (_capacity . bv)) 2)) ;; bignum
              (write-long (ash (elt (_capacity . bv) 0) 0) s)
              (write-long (ash (elt (_capacity . bv) 1) -1) s))
             ((and (class _capacity) (= (length (_capacity . bv)) 1)) ;; big1
              (write-long (elt (_capacity . bv) 0) s)
              (write-long (if (>= _capacity 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _capacity s)(write-long (if (>= _capacity 0) 0 #xffffffff) s)))
     ;; uint64 _used
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _used (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _used) (= (length (_used . bv)) 2)) ;; bignum
              (write-long (ash (elt (_used . bv) 0) 0) s)
              (write-long (ash (elt (_used . bv) 1) -1) s))
             ((and (class _used) (= (length (_used . bv)) 1)) ;; big1
              (write-long (elt (_used . bv) 0) s)
              (write-long (if (>= _used 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _used s)(write-long (if (>= _used 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _path
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _path (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint64 _capacity
     
#+(or :alpha :irix6 :x86_64)
      (setf _capacity (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _capacity (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; uint64 _used
     
#+(or :alpha :irix6 :x86_64)
      (setf _used (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _used (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get ff_msgs::DiskState :md5sum-) "d5dcd3cfca06139f3d404ba9fc6b35be")
(setf (get ff_msgs::DiskState :datatype-) "ff_msgs/DiskState")
(setf (get ff_msgs::DiskState :definition-)
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
# This message describes the state of a filesystem within astrobee
# Based off of DiskState from rapid::ext::astrobee

string path       # The pathname of the file within the mounted filesystem
uint64 capacity   # The size of the filesystem
uint64 used       # The amount of the filesystem being used

")



(provide :ff_msgs/DiskState "d5dcd3cfca06139f3d404ba9fc6b35be")


