;; Auto-generated. Do not edit!


(when (boundp 'coordinator::StatusPrimary)
  (if (not (find-package "COORDINATOR"))
    (make-package "COORDINATOR"))
  (shadow 'StatusPrimary (find-package "COORDINATOR")))
(unless (find-package "COORDINATOR::STATUSPRIMARY")
  (make-package "COORDINATOR::STATUSPRIMARY"))

(in-package "ROS")
;;//! \htmlinclude StatusPrimary.msg.html


(defclass coordinator::StatusPrimary
  :super ros::object
  :slots (_stamp _test_number _default_control _flight_mode _test_finished _coord_ok _control_mode _regulate_finished _description ))

(defmethod coordinator::StatusPrimary
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:test_number __test_number) 0)
    ((:default_control __default_control) nil)
    ((:flight_mode __flight_mode) "")
    ((:test_finished __test_finished) nil)
    ((:coord_ok __coord_ok) nil)
    ((:control_mode __control_mode) "")
    ((:regulate_finished __regulate_finished) nil)
    ((:description __description) "")
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _test_number (round __test_number))
   (setq _default_control __default_control)
   (setq _flight_mode (string __flight_mode))
   (setq _test_finished __test_finished)
   (setq _coord_ok __coord_ok)
   (setq _control_mode (string __control_mode))
   (setq _regulate_finished __regulate_finished)
   (setq _description (string __description))
   self)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:test_number
   (&optional __test_number)
   (if __test_number (setq _test_number __test_number)) _test_number)
  (:default_control
   (&optional __default_control)
   (if __default_control (setq _default_control __default_control)) _default_control)
  (:flight_mode
   (&optional __flight_mode)
   (if __flight_mode (setq _flight_mode __flight_mode)) _flight_mode)
  (:test_finished
   (&optional __test_finished)
   (if __test_finished (setq _test_finished __test_finished)) _test_finished)
  (:coord_ok
   (&optional __coord_ok)
   (if __coord_ok (setq _coord_ok __coord_ok)) _coord_ok)
  (:control_mode
   (&optional __control_mode)
   (if __control_mode (setq _control_mode __control_mode)) _control_mode)
  (:regulate_finished
   (&optional __regulate_finished)
   (if __regulate_finished (setq _regulate_finished __regulate_finished)) _regulate_finished)
  (:description
   (&optional __description)
   (if __description (setq _description __description)) _description)
  (:serialization-length
   ()
   (+
    ;; time _stamp
    8
    ;; int32 _test_number
    4
    ;; bool _default_control
    1
    ;; string _flight_mode
    4 (length _flight_mode)
    ;; bool _test_finished
    1
    ;; bool _coord_ok
    1
    ;; string _control_mode
    4 (length _control_mode)
    ;; bool _regulate_finished
    1
    ;; string _description
    4 (length _description)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; int32 _test_number
       (write-long _test_number s)
     ;; bool _default_control
       (if _default_control (write-byte -1 s) (write-byte 0 s))
     ;; string _flight_mode
       (write-long (length _flight_mode) s) (princ _flight_mode s)
     ;; bool _test_finished
       (if _test_finished (write-byte -1 s) (write-byte 0 s))
     ;; bool _coord_ok
       (if _coord_ok (write-byte -1 s) (write-byte 0 s))
     ;; string _control_mode
       (write-long (length _control_mode) s) (princ _control_mode s)
     ;; bool _regulate_finished
       (if _regulate_finished (write-byte -1 s) (write-byte 0 s))
     ;; string _description
       (write-long (length _description) s) (princ _description s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _test_number
     (setq _test_number (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _default_control
     (setq _default_control (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _flight_mode
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _flight_mode (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _test_finished
     (setq _test_finished (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _coord_ok
     (setq _coord_ok (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _control_mode
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _control_mode (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _regulate_finished
     (setq _regulate_finished (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _description
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _description (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get coordinator::StatusPrimary :md5sum-) "7b1f7333b81ba92ccdc2c10a9400c075")
(setf (get coordinator::StatusPrimary :datatype-) "coordinator/StatusPrimary")
(setf (get coordinator::StatusPrimary :definition-)
      "time stamp

# base (shared) values
int32 test_number
bool default_control
string flight_mode
bool test_finished
bool coord_ok

# StatusPrimary
string control_mode  # did something break?
bool regulate_finished
string description  # for telemetry description

")



(provide :coordinator/StatusPrimary "7b1f7333b81ba92ccdc2c10a9400c075")


