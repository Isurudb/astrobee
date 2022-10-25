;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::Trigger)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'Trigger (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::TRIGGER")
  (make-package "FF_MSGS::TRIGGER"))
(unless (find-package "FF_MSGS::TRIGGERREQUEST")
  (make-package "FF_MSGS::TRIGGERREQUEST"))
(unless (find-package "FF_MSGS::TRIGGERRESPONSE")
  (make-package "FF_MSGS::TRIGGERRESPONSE"))

(in-package "ROS")





(intern "*UNKNOWN*" (find-package "FF_MSGS::TRIGGERREQUEST"))
(shadow '*UNKNOWN* (find-package "FF_MSGS::TRIGGERREQUEST"))
(defconstant ff_msgs::TriggerRequest::*UNKNOWN* 0)
(intern "*RESTART*" (find-package "FF_MSGS::TRIGGERREQUEST"))
(shadow '*RESTART* (find-package "FF_MSGS::TRIGGERREQUEST"))
(defconstant ff_msgs::TriggerRequest::*RESTART* 1)
(intern "*SLEEP*" (find-package "FF_MSGS::TRIGGERREQUEST"))
(shadow '*SLEEP* (find-package "FF_MSGS::TRIGGERREQUEST"))
(defconstant ff_msgs::TriggerRequest::*SLEEP* 2)
(intern "*WAKEUP*" (find-package "FF_MSGS::TRIGGERREQUEST"))
(shadow '*WAKEUP* (find-package "FF_MSGS::TRIGGERREQUEST"))
(defconstant ff_msgs::TriggerRequest::*WAKEUP* 3)
(defclass ff_msgs::TriggerRequest
  :super ros::object
  :slots (_event ))

(defmethod ff_msgs::TriggerRequest
  (:init
   (&key
    ((:event __event) 0)
    )
   (send-super :init)
   (setq _event (round __event))
   self)
  (:event
   (&optional __event)
   (if __event (setq _event __event)) _event)
  (:serialization-length
   ()
   (+
    ;; uint8 _event
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _event
       (write-byte _event s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _event
     (setq _event (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass ff_msgs::TriggerResponse
  :super ros::object
  :slots ())

(defmethod ff_msgs::TriggerResponse
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

(defclass ff_msgs::Trigger
  :super ros::object
  :slots ())

(setf (get ff_msgs::Trigger :md5sum-) "15235914233b414e535b7900827f7f14")
(setf (get ff_msgs::Trigger :datatype-) "ff_msgs/Trigger")
(setf (get ff_msgs::Trigger :request) ff_msgs::TriggerRequest)
(setf (get ff_msgs::Trigger :response) ff_msgs::TriggerResponse)

(defmethod ff_msgs::TriggerRequest
  (:response () (instance ff_msgs::TriggerResponse :init)))

(setf (get ff_msgs::TriggerRequest :md5sum-) "15235914233b414e535b7900827f7f14")
(setf (get ff_msgs::TriggerRequest :datatype-) "ff_msgs/TriggerRequest")
(setf (get ff_msgs::TriggerRequest :definition-)
      "


















uint8 event
uint8 UNKNOWN = 0
uint8 RESTART = 1
uint8 SLEEP   = 2
uint8 WAKEUP  = 3
---

")

(setf (get ff_msgs::TriggerResponse :md5sum-) "15235914233b414e535b7900827f7f14")
(setf (get ff_msgs::TriggerResponse :datatype-) "ff_msgs/TriggerResponse")
(setf (get ff_msgs::TriggerResponse :definition-)
      "


















uint8 event
uint8 UNKNOWN = 0
uint8 RESTART = 1
uint8 SLEEP   = 2
uint8 WAKEUP  = 3
---

")



(provide :ff_msgs/Trigger "15235914233b414e535b7900827f7f14")


