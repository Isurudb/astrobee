;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::VisualeyezConfig)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'VisualeyezConfig (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::VISUALEYEZCONFIG")
  (make-package "FF_MSGS::VISUALEYEZCONFIG"))
(unless (find-package "FF_MSGS::VISUALEYEZCONFIGREQUEST")
  (make-package "FF_MSGS::VISUALEYEZCONFIGREQUEST"))
(unless (find-package "FF_MSGS::VISUALEYEZCONFIGRESPONSE")
  (make-package "FF_MSGS::VISUALEYEZCONFIGRESPONSE"))

(in-package "ROS")





(intern "*TRACK*" (find-package "FF_MSGS::VISUALEYEZCONFIGREQUEST"))
(shadow '*TRACK* (find-package "FF_MSGS::VISUALEYEZCONFIGREQUEST"))
(defconstant ff_msgs::VisualeyezConfigRequest::*TRACK* 0)
(intern "*RECORD*" (find-package "FF_MSGS::VISUALEYEZCONFIGREQUEST"))
(shadow '*RECORD* (find-package "FF_MSGS::VISUALEYEZCONFIGREQUEST"))
(defconstant ff_msgs::VisualeyezConfigRequest::*RECORD* 1)
(intern "*CALIBRATE*" (find-package "FF_MSGS::VISUALEYEZCONFIGREQUEST"))
(shadow '*CALIBRATE* (find-package "FF_MSGS::VISUALEYEZCONFIGREQUEST"))
(defconstant ff_msgs::VisualeyezConfigRequest::*CALIBRATE* 2)
(intern "*LOAD*" (find-package "FF_MSGS::VISUALEYEZCONFIGREQUEST"))
(shadow '*LOAD* (find-package "FF_MSGS::VISUALEYEZCONFIGREQUEST"))
(defconstant ff_msgs::VisualeyezConfigRequest::*LOAD* 3)
(intern "*SAVE*" (find-package "FF_MSGS::VISUALEYEZCONFIGREQUEST"))
(shadow '*SAVE* (find-package "FF_MSGS::VISUALEYEZCONFIGREQUEST"))
(defconstant ff_msgs::VisualeyezConfigRequest::*SAVE* 4)
(defclass ff_msgs::VisualeyezConfigRequest
  :super ros::object
  :slots (_action _name _pub_tf ))

(defmethod ff_msgs::VisualeyezConfigRequest
  (:init
   (&key
    ((:action __action) 0)
    ((:name __name) "")
    ((:pub_tf __pub_tf) nil)
    )
   (send-super :init)
   (setq _action (round __action))
   (setq _name (string __name))
   (setq _pub_tf __pub_tf)
   self)
  (:action
   (&optional __action)
   (if __action (setq _action __action)) _action)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:pub_tf
   (&optional __pub_tf)
   (if __pub_tf (setq _pub_tf __pub_tf)) _pub_tf)
  (:serialization-length
   ()
   (+
    ;; uint8 _action
    1
    ;; string _name
    4 (length _name)
    ;; bool _pub_tf
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _action
       (write-byte _action s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; bool _pub_tf
       (if _pub_tf (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _action
     (setq _action (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _pub_tf
     (setq _pub_tf (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ff_msgs::VisualeyezConfigResponse
  :super ros::object
  :slots (_success ))

(defmethod ff_msgs::VisualeyezConfigResponse
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

(defclass ff_msgs::VisualeyezConfig
  :super ros::object
  :slots ())

(setf (get ff_msgs::VisualeyezConfig :md5sum-) "79a3f4aad18c09360c2499f911dea41f")
(setf (get ff_msgs::VisualeyezConfig :datatype-) "ff_msgs/VisualeyezConfig")
(setf (get ff_msgs::VisualeyezConfig :request) ff_msgs::VisualeyezConfigRequest)
(setf (get ff_msgs::VisualeyezConfig :response) ff_msgs::VisualeyezConfigResponse)

(defmethod ff_msgs::VisualeyezConfigRequest
  (:response () (instance ff_msgs::VisualeyezConfigResponse :init)))

(setf (get ff_msgs::VisualeyezConfigRequest :md5sum-) "79a3f4aad18c09360c2499f911dea41f")
(setf (get ff_msgs::VisualeyezConfigRequest :datatype-) "ff_msgs/VisualeyezConfigRequest")
(setf (get ff_msgs::VisualeyezConfigRequest :definition-)
      "


















uint8 action
uint8 TRACK       = 0
uint8 RECORD      = 1
uint8 CALIBRATE   = 2
uint8 LOAD        = 3
uint8 SAVE        = 4


string name


bool pub_tf

---

bool success

")

(setf (get ff_msgs::VisualeyezConfigResponse :md5sum-) "79a3f4aad18c09360c2499f911dea41f")
(setf (get ff_msgs::VisualeyezConfigResponse :datatype-) "ff_msgs/VisualeyezConfigResponse")
(setf (get ff_msgs::VisualeyezConfigResponse :definition-)
      "


















uint8 action
uint8 TRACK       = 0
uint8 RECORD      = 1
uint8 CALIBRATE   = 2
uint8 LOAD        = 3
uint8 SAVE        = 4


string name


bool pub_tf

---

bool success

")



(provide :ff_msgs/VisualeyezConfig "79a3f4aad18c09360c2499f911dea41f")


