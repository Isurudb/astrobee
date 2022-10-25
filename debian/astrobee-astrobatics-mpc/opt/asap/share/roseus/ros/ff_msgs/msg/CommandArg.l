;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::CommandArg)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'CommandArg (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::COMMANDARG")
  (make-package "FF_MSGS::COMMANDARG"))

(in-package "ROS")
;;//! \htmlinclude CommandArg.msg.html


(intern "*DATA_TYPE_BOOL*" (find-package "FF_MSGS::COMMANDARG"))
(shadow '*DATA_TYPE_BOOL* (find-package "FF_MSGS::COMMANDARG"))
(defconstant ff_msgs::CommandArg::*DATA_TYPE_BOOL* 0)
(intern "*DATA_TYPE_DOUBLE*" (find-package "FF_MSGS::COMMANDARG"))
(shadow '*DATA_TYPE_DOUBLE* (find-package "FF_MSGS::COMMANDARG"))
(defconstant ff_msgs::CommandArg::*DATA_TYPE_DOUBLE* 1)
(intern "*DATA_TYPE_FLOAT*" (find-package "FF_MSGS::COMMANDARG"))
(shadow '*DATA_TYPE_FLOAT* (find-package "FF_MSGS::COMMANDARG"))
(defconstant ff_msgs::CommandArg::*DATA_TYPE_FLOAT* 2)
(intern "*DATA_TYPE_INT*" (find-package "FF_MSGS::COMMANDARG"))
(shadow '*DATA_TYPE_INT* (find-package "FF_MSGS::COMMANDARG"))
(defconstant ff_msgs::CommandArg::*DATA_TYPE_INT* 3)
(intern "*DATA_TYPE_LONGLONG*" (find-package "FF_MSGS::COMMANDARG"))
(shadow '*DATA_TYPE_LONGLONG* (find-package "FF_MSGS::COMMANDARG"))
(defconstant ff_msgs::CommandArg::*DATA_TYPE_LONGLONG* 4)
(intern "*DATA_TYPE_STRING*" (find-package "FF_MSGS::COMMANDARG"))
(shadow '*DATA_TYPE_STRING* (find-package "FF_MSGS::COMMANDARG"))
(defconstant ff_msgs::CommandArg::*DATA_TYPE_STRING* 5)
(intern "*DATA_TYPE_VEC3D*" (find-package "FF_MSGS::COMMANDARG"))
(shadow '*DATA_TYPE_VEC3D* (find-package "FF_MSGS::COMMANDARG"))
(defconstant ff_msgs::CommandArg::*DATA_TYPE_VEC3D* 6)
(intern "*DATA_TYPE_MAT33F*" (find-package "FF_MSGS::COMMANDARG"))
(shadow '*DATA_TYPE_MAT33F* (find-package "FF_MSGS::COMMANDARG"))
(defconstant ff_msgs::CommandArg::*DATA_TYPE_MAT33F* 7)
(defclass ff_msgs::CommandArg
  :super ros::object
  :slots (_data_type _b _d _f _i _ll _s _vec3d _mat33f ))

(defmethod ff_msgs::CommandArg
  (:init
   (&key
    ((:data_type __data_type) 0)
    ((:b __b) nil)
    ((:d __d) 0.0)
    ((:f __f) 0.0)
    ((:i __i) 0)
    ((:ll __ll) 0)
    ((:s __s) "")
    ((:vec3d __vec3d) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:mat33f __mat33f) (make-array 9 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _data_type (round __data_type))
   (setq _b __b)
   (setq _d (float __d))
   (setq _f (float __f))
   (setq _i (round __i))
   (setq _ll (round __ll))
   (setq _s (string __s))
   (setq _vec3d __vec3d)
   (setq _mat33f __mat33f)
   self)
  (:data_type
   (&optional __data_type)
   (if __data_type (setq _data_type __data_type)) _data_type)
  (:b
   (&optional __b)
   (if __b (setq _b __b)) _b)
  (:d
   (&optional __d)
   (if __d (setq _d __d)) _d)
  (:f
   (&optional __f)
   (if __f (setq _f __f)) _f)
  (:i
   (&optional __i)
   (if __i (setq _i __i)) _i)
  (:ll
   (&optional __ll)
   (if __ll (setq _ll __ll)) _ll)
  (:s
   (&optional __s)
   (if __s (setq _s __s)) _s)
  (:vec3d
   (&optional __vec3d)
   (if __vec3d (setq _vec3d __vec3d)) _vec3d)
  (:mat33f
   (&optional __mat33f)
   (if __mat33f (setq _mat33f __mat33f)) _mat33f)
  (:serialization-length
   ()
   (+
    ;; uint8 _data_type
    1
    ;; bool _b
    1
    ;; float64 _d
    8
    ;; float32 _f
    4
    ;; int32 _i
    4
    ;; int64 _ll
    8
    ;; string _s
    4 (length _s)
    ;; float64[3] _vec3d
    (* 8    3)
    ;; float32[9] _mat33f
    (* 4    9)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _data_type
       (write-byte _data_type s)
     ;; bool _b
       (if _b (write-byte -1 s) (write-byte 0 s))
     ;; float64 _d
       (sys::poke _d (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _f
       (sys::poke _f (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _i
       (write-long _i s)
     ;; int64 _ll
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _ll (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _ll) (= (length (_ll . bv)) 2)) ;; bignum
              (write-long (ash (elt (_ll . bv) 0) 0) s)
              (write-long (ash (elt (_ll . bv) 1) -1) s))
             ((and (class _ll) (= (length (_ll . bv)) 1)) ;; big1
              (write-long (elt (_ll . bv) 0) s)
              (write-long (if (>= _ll 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _ll s)(write-long (if (>= _ll 0) 0 #xffffffff) s)))
     ;; string _s
       (write-long (length _s) s) (princ _s s)
     ;; float64[3] _vec3d
     (dotimes (i 3)
       (sys::poke (elt _vec3d i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float32[9] _mat33f
     (dotimes (i 9)
       (sys::poke (elt _mat33f i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _data_type
     (setq _data_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _b
     (setq _b (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _d
     (setq _d (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _f
     (setq _f (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _i
     (setq _i (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int64 _ll
#+(or :alpha :irix6 :x86_64)
      (setf _ll (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _ll (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; string _s
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _s (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64[3] _vec3d
   (dotimes (i (length _vec3d))
     (setf (elt _vec3d i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float32[9] _mat33f
   (dotimes (i (length _mat33f))
     (setf (elt _mat33f i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get ff_msgs::CommandArg :md5sum-) "c64f399f685551792b2e185eb2878830")
(setf (get ff_msgs::CommandArg :datatype-) "ff_msgs/CommandArg")
(setf (get ff_msgs::CommandArg :definition-)
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
# An argument to a command sent through RAPID
#
# Note that this is approximating a union in DDS. However, this is an
# inefficient union, and thus each instance will take up at least 89 bytes.
# However, even with the maximum of 16 arguments to a command, we only have
# about 1k extra data. I, tfmorse, am ok with that. Commands are rarely sent.

uint8 DATA_TYPE_BOOL     = 0
uint8 DATA_TYPE_DOUBLE   = 1
uint8 DATA_TYPE_FLOAT    = 2
uint8 DATA_TYPE_INT      = 3
uint8 DATA_TYPE_LONGLONG = 4
uint8 DATA_TYPE_STRING   = 5
uint8 DATA_TYPE_VEC3d    = 6
uint8 DATA_TYPE_MAT33f   = 7

uint8 data_type

bool b
float64 d
float32 f
int32 i
int64 ll
string s
float64[3] vec3d
float32[9] mat33f


")



(provide :ff_msgs/CommandArg "c64f399f685551792b2e185eb2878830")


