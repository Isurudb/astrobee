;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::SetState)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'SetState (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::SETSTATE")
  (make-package "FF_MSGS::SETSTATE"))
(unless (find-package "FF_MSGS::SETSTATEREQUEST")
  (make-package "FF_MSGS::SETSTATEREQUEST"))
(unless (find-package "FF_MSGS::SETSTATERESPONSE")
  (make-package "FF_MSGS::SETSTATERESPONSE"))

(in-package "ROS")





(defclass ff_msgs::SetStateRequest
  :super ros::object
  :slots (_state ))

(defmethod ff_msgs::SetStateRequest
  (:init
   (&key
    ((:state __state) 0)
    )
   (send-super :init)
   (setq _state (round __state))
   self)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; int32 _state
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _state
       (write-long _state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _state
     (setq _state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass ff_msgs::SetStateResponse
  :super ros::object
  :slots (_success ))

(defmethod ff_msgs::SetStateResponse
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

(defclass ff_msgs::SetState
  :super ros::object
  :slots ())

(setf (get ff_msgs::SetState :md5sum-) "6d0ec020603882606d0a12cc504ac31c")
(setf (get ff_msgs::SetState :datatype-) "ff_msgs/SetState")
(setf (get ff_msgs::SetState :request) ff_msgs::SetStateRequest)
(setf (get ff_msgs::SetState :response) ff_msgs::SetStateResponse)

(defmethod ff_msgs::SetStateRequest
  (:response () (instance ff_msgs::SetStateResponse :init)))

(setf (get ff_msgs::SetStateRequest :md5sum-) "6d0ec020603882606d0a12cc504ac31c")
(setf (get ff_msgs::SetStateRequest :datatype-) "ff_msgs/SetStateRequest")
(setf (get ff_msgs::SetStateRequest :definition-)
      "


















int32 state
---
bool success
")

(setf (get ff_msgs::SetStateResponse :md5sum-) "6d0ec020603882606d0a12cc504ac31c")
(setf (get ff_msgs::SetStateResponse :datatype-) "ff_msgs/SetStateResponse")
(setf (get ff_msgs::SetStateResponse :definition-)
      "


















int32 state
---
bool success
")



(provide :ff_msgs/SetState "6d0ec020603882606d0a12cc504ac31c")


