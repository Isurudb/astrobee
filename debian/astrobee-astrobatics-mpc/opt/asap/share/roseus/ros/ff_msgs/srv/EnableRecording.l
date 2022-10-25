;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::EnableRecording)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'EnableRecording (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::ENABLERECORDING")
  (make-package "FF_MSGS::ENABLERECORDING"))
(unless (find-package "FF_MSGS::ENABLERECORDINGREQUEST")
  (make-package "FF_MSGS::ENABLERECORDINGREQUEST"))
(unless (find-package "FF_MSGS::ENABLERECORDINGRESPONSE")
  (make-package "FF_MSGS::ENABLERECORDINGRESPONSE"))

(in-package "ROS")





(defclass ff_msgs::EnableRecordingRequest
  :super ros::object
  :slots (_enable _bag_description ))

(defmethod ff_msgs::EnableRecordingRequest
  (:init
   (&key
    ((:enable __enable) nil)
    ((:bag_description __bag_description) "")
    )
   (send-super :init)
   (setq _enable __enable)
   (setq _bag_description (string __bag_description))
   self)
  (:enable
   (&optional __enable)
   (if __enable (setq _enable __enable)) _enable)
  (:bag_description
   (&optional __bag_description)
   (if __bag_description (setq _bag_description __bag_description)) _bag_description)
  (:serialization-length
   ()
   (+
    ;; bool _enable
    1
    ;; string _bag_description
    4 (length _bag_description)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _enable
       (if _enable (write-byte -1 s) (write-byte 0 s))
     ;; string _bag_description
       (write-long (length _bag_description) s) (princ _bag_description s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _enable
     (setq _enable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _bag_description
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _bag_description (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ff_msgs::EnableRecordingResponse
  :super ros::object
  :slots (_success _status ))

(defmethod ff_msgs::EnableRecordingResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:status __status) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _status (string __status))
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _status
    4 (length _status)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _status
       (write-long (length _status) s) (princ _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ff_msgs::EnableRecording
  :super ros::object
  :slots ())

(setf (get ff_msgs::EnableRecording :md5sum-) "68fd83cd501355d809d1d0420334c998")
(setf (get ff_msgs::EnableRecording :datatype-) "ff_msgs/EnableRecording")
(setf (get ff_msgs::EnableRecording :request) ff_msgs::EnableRecordingRequest)
(setf (get ff_msgs::EnableRecording :response) ff_msgs::EnableRecordingResponse)

(defmethod ff_msgs::EnableRecordingRequest
  (:response () (instance ff_msgs::EnableRecordingResponse :init)))

(setf (get ff_msgs::EnableRecordingRequest :md5sum-) "68fd83cd501355d809d1d0420334c998")
(setf (get ff_msgs::EnableRecordingRequest :datatype-) "ff_msgs/EnableRecordingRequest")
(setf (get ff_msgs::EnableRecordingRequest :definition-)
      "


















bool enable
string bag_description
---
bool success
string status

")

(setf (get ff_msgs::EnableRecordingResponse :md5sum-) "68fd83cd501355d809d1d0420334c998")
(setf (get ff_msgs::EnableRecordingResponse :datatype-) "ff_msgs/EnableRecordingResponse")
(setf (get ff_msgs::EnableRecordingResponse :definition-)
      "


















bool enable
string bag_description
---
bool success
string status

")



(provide :ff_msgs/EnableRecording "68fd83cd501355d809d1d0420334c998")


