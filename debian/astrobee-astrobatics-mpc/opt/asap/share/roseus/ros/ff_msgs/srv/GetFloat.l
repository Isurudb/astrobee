;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::GetFloat)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'GetFloat (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::GETFLOAT")
  (make-package "FF_MSGS::GETFLOAT"))
(unless (find-package "FF_MSGS::GETFLOATREQUEST")
  (make-package "FF_MSGS::GETFLOATREQUEST"))
(unless (find-package "FF_MSGS::GETFLOATRESPONSE")
  (make-package "FF_MSGS::GETFLOATRESPONSE"))

(in-package "ROS")





(defclass ff_msgs::GetFloatRequest
  :super ros::object
  :slots ())

(defmethod ff_msgs::GetFloatRequest
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

(defclass ff_msgs::GetFloatResponse
  :super ros::object
  :slots (_data _success ))

(defmethod ff_msgs::GetFloatResponse
  (:init
   (&key
    ((:data __data) 0.0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _data (float __data))
   (setq _success __success)
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; float64 _data
    8
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _data
       (sys::poke _data (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _data
     (setq _data (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ff_msgs::GetFloat
  :super ros::object
  :slots ())

(setf (get ff_msgs::GetFloat :md5sum-) "283c964108997a34d3fae942dada5b32")
(setf (get ff_msgs::GetFloat :datatype-) "ff_msgs/GetFloat")
(setf (get ff_msgs::GetFloat :request) ff_msgs::GetFloatRequest)
(setf (get ff_msgs::GetFloat :response) ff_msgs::GetFloatResponse)

(defmethod ff_msgs::GetFloatRequest
  (:response () (instance ff_msgs::GetFloatResponse :init)))

(setf (get ff_msgs::GetFloatRequest :md5sum-) "283c964108997a34d3fae942dada5b32")
(setf (get ff_msgs::GetFloatRequest :datatype-) "ff_msgs/GetFloatRequest")
(setf (get ff_msgs::GetFloatRequest :definition-)
      "


















---
float64 data
bool success

")

(setf (get ff_msgs::GetFloatResponse :md5sum-) "283c964108997a34d3fae942dada5b32")
(setf (get ff_msgs::GetFloatResponse :datatype-) "ff_msgs/GetFloatResponse")
(setf (get ff_msgs::GetFloatResponse :definition-)
      "


















---
float64 data
bool success

")



(provide :ff_msgs/GetFloat "283c964108997a34d3fae942dada5b32")


