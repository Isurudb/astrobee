;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::SetFloat)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'SetFloat (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::SETFLOAT")
  (make-package "FF_MSGS::SETFLOAT"))
(unless (find-package "FF_MSGS::SETFLOATREQUEST")
  (make-package "FF_MSGS::SETFLOATREQUEST"))
(unless (find-package "FF_MSGS::SETFLOATRESPONSE")
  (make-package "FF_MSGS::SETFLOATRESPONSE"))

(in-package "ROS")





(defclass ff_msgs::SetFloatRequest
  :super ros::object
  :slots (_data ))

(defmethod ff_msgs::SetFloatRequest
  (:init
   (&key
    ((:data __data) 0.0)
    )
   (send-super :init)
   (setq _data (float __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; float64 _data
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _data
       (sys::poke _data (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _data
     (setq _data (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass ff_msgs::SetFloatResponse
  :super ros::object
  :slots (_success ))

(defmethod ff_msgs::SetFloatResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ff_msgs::SetFloat
  :super ros::object
  :slots ())

(setf (get ff_msgs::SetFloat :md5sum-) "6dffcb6acc6bec80315e1c470ea1bca9")
(setf (get ff_msgs::SetFloat :datatype-) "ff_msgs/SetFloat")
(setf (get ff_msgs::SetFloat :request) ff_msgs::SetFloatRequest)
(setf (get ff_msgs::SetFloat :response) ff_msgs::SetFloatResponse)

(defmethod ff_msgs::SetFloatRequest
  (:response () (instance ff_msgs::SetFloatResponse :init)))

(setf (get ff_msgs::SetFloatRequest :md5sum-) "6dffcb6acc6bec80315e1c470ea1bca9")
(setf (get ff_msgs::SetFloatRequest :datatype-) "ff_msgs/SetFloatRequest")
(setf (get ff_msgs::SetFloatRequest :definition-)
      "


















float64 data
---
bool success

")

(setf (get ff_msgs::SetFloatResponse :md5sum-) "6dffcb6acc6bec80315e1c470ea1bca9")
(setf (get ff_msgs::SetFloatResponse :datatype-) "ff_msgs/SetFloatResponse")
(setf (get ff_msgs::SetFloatResponse :definition-)
      "


















float64 data
---
bool success

")



(provide :ff_msgs/SetFloat "6dffcb6acc6bec80315e1c470ea1bca9")


