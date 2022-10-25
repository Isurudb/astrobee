;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::DockResult)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'DockResult (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::DOCKRESULT")
  (make-package "FF_MSGS::DOCKRESULT"))

(in-package "ROS")
;;//! \htmlinclude DockResult.msg.html


(intern "*CANCELLED*" (find-package "FF_MSGS::DOCKRESULT"))
(shadow '*CANCELLED* (find-package "FF_MSGS::DOCKRESULT"))
(defconstant ff_msgs::DockResult::*CANCELLED* 5)
(intern "*ALREADY_DOCKED*" (find-package "FF_MSGS::DOCKRESULT"))
(shadow '*ALREADY_DOCKED* (find-package "FF_MSGS::DOCKRESULT"))
(defconstant ff_msgs::DockResult::*ALREADY_DOCKED* 4)
(intern "*ALREADY_UNDOCKED*" (find-package "FF_MSGS::DOCKRESULT"))
(shadow '*ALREADY_UNDOCKED* (find-package "FF_MSGS::DOCKRESULT"))
(defconstant ff_msgs::DockResult::*ALREADY_UNDOCKED* 3)
(intern "*UNDOCKED*" (find-package "FF_MSGS::DOCKRESULT"))
(shadow '*UNDOCKED* (find-package "FF_MSGS::DOCKRESULT"))
(defconstant ff_msgs::DockResult::*UNDOCKED* 2)
(intern "*DOCKED*" (find-package "FF_MSGS::DOCKRESULT"))
(shadow '*DOCKED* (find-package "FF_MSGS::DOCKRESULT"))
(defconstant ff_msgs::DockResult::*DOCKED* 1)
(intern "*PREEMPTED*" (find-package "FF_MSGS::DOCKRESULT"))
(shadow '*PREEMPTED* (find-package "FF_MSGS::DOCKRESULT"))
(defconstant ff_msgs::DockResult::*PREEMPTED* 0)
(intern "*INVALID_COMMAND*" (find-package "FF_MSGS::DOCKRESULT"))
(shadow '*INVALID_COMMAND* (find-package "FF_MSGS::DOCKRESULT"))
(defconstant ff_msgs::DockResult::*INVALID_COMMAND* -1)
(intern "*INVALID_BERTH*" (find-package "FF_MSGS::DOCKRESULT"))
(shadow '*INVALID_BERTH* (find-package "FF_MSGS::DOCKRESULT"))
(defconstant ff_msgs::DockResult::*INVALID_BERTH* -2)
(intern "*NOT_IN_UNDOCKED_STATE*" (find-package "FF_MSGS::DOCKRESULT"))
(shadow '*NOT_IN_UNDOCKED_STATE* (find-package "FF_MSGS::DOCKRESULT"))
(defconstant ff_msgs::DockResult::*NOT_IN_UNDOCKED_STATE* -3)
(intern "*NOT_IN_DOCKED_STATE*" (find-package "FF_MSGS::DOCKRESULT"))
(shadow '*NOT_IN_DOCKED_STATE* (find-package "FF_MSGS::DOCKRESULT"))
(defconstant ff_msgs::DockResult::*NOT_IN_DOCKED_STATE* -4)
(intern "*SWITCH_TO_ML_FAILED*" (find-package "FF_MSGS::DOCKRESULT"))
(shadow '*SWITCH_TO_ML_FAILED* (find-package "FF_MSGS::DOCKRESULT"))
(defconstant ff_msgs::DockResult::*SWITCH_TO_ML_FAILED* -5)
(intern "*SWITCH_TO_AR_FAILED*" (find-package "FF_MSGS::DOCKRESULT"))
(shadow '*SWITCH_TO_AR_FAILED* (find-package "FF_MSGS::DOCKRESULT"))
(defconstant ff_msgs::DockResult::*SWITCH_TO_AR_FAILED* -6)
(intern "*SWITCH_TO_NO_FAILED*" (find-package "FF_MSGS::DOCKRESULT"))
(shadow '*SWITCH_TO_NO_FAILED* (find-package "FF_MSGS::DOCKRESULT"))
(defconstant ff_msgs::DockResult::*SWITCH_TO_NO_FAILED* -7)
(intern "*PREP_DISABLE_FAILED*" (find-package "FF_MSGS::DOCKRESULT"))
(shadow '*PREP_DISABLE_FAILED* (find-package "FF_MSGS::DOCKRESULT"))
(defconstant ff_msgs::DockResult::*PREP_DISABLE_FAILED* -8)
(intern "*PREP_ENABLE_FAILED*" (find-package "FF_MSGS::DOCKRESULT"))
(shadow '*PREP_ENABLE_FAILED* (find-package "FF_MSGS::DOCKRESULT"))
(defconstant ff_msgs::DockResult::*PREP_ENABLE_FAILED* -9)
(intern "*MOTION_APPROACH_FAILED*" (find-package "FF_MSGS::DOCKRESULT"))
(shadow '*MOTION_APPROACH_FAILED* (find-package "FF_MSGS::DOCKRESULT"))
(defconstant ff_msgs::DockResult::*MOTION_APPROACH_FAILED* -10)
(intern "*MOTION_COMPLETE_FAILED*" (find-package "FF_MSGS::DOCKRESULT"))
(shadow '*MOTION_COMPLETE_FAILED* (find-package "FF_MSGS::DOCKRESULT"))
(defconstant ff_msgs::DockResult::*MOTION_COMPLETE_FAILED* -11)
(intern "*MOTION_ATTACHED_FAILED*" (find-package "FF_MSGS::DOCKRESULT"))
(shadow '*MOTION_ATTACHED_FAILED* (find-package "FF_MSGS::DOCKRESULT"))
(defconstant ff_msgs::DockResult::*MOTION_ATTACHED_FAILED* -12)
(intern "*EPS_UNDOCK_FAILED*" (find-package "FF_MSGS::DOCKRESULT"))
(shadow '*EPS_UNDOCK_FAILED* (find-package "FF_MSGS::DOCKRESULT"))
(defconstant ff_msgs::DockResult::*EPS_UNDOCK_FAILED* -13)
(intern "*EPS_DOCK_FAILED*" (find-package "FF_MSGS::DOCKRESULT"))
(shadow '*EPS_DOCK_FAILED* (find-package "FF_MSGS::DOCKRESULT"))
(defconstant ff_msgs::DockResult::*EPS_DOCK_FAILED* -14)
(intern "*TOO_FAR_AWAY_FROM_APPROACH*" (find-package "FF_MSGS::DOCKRESULT"))
(shadow '*TOO_FAR_AWAY_FROM_APPROACH* (find-package "FF_MSGS::DOCKRESULT"))
(defconstant ff_msgs::DockResult::*TOO_FAR_AWAY_FROM_APPROACH* -15)
(defclass ff_msgs::DockResult
  :super ros::object
  :slots (_response _fsm_result ))

(defmethod ff_msgs::DockResult
  (:init
   (&key
    ((:response __response) 0)
    ((:fsm_result __fsm_result) "")
    )
   (send-super :init)
   (setq _response (round __response))
   (setq _fsm_result (string __fsm_result))
   self)
  (:response
   (&optional __response)
   (if __response (setq _response __response)) _response)
  (:fsm_result
   (&optional __fsm_result)
   (if __fsm_result (setq _fsm_result __fsm_result)) _fsm_result)
  (:serialization-length
   ()
   (+
    ;; int32 _response
    4
    ;; string _fsm_result
    4 (length _fsm_result)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _response
       (write-long _response s)
     ;; string _fsm_result
       (write-long (length _fsm_result) s) (princ _fsm_result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _response
     (setq _response (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _fsm_result
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _fsm_result (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get ff_msgs::DockResult :md5sum-) "0cc69ac3a301c7996578d2ee3e9b92a6")
(setf (get ff_msgs::DockResult :datatype-) "ff_msgs/DockResult")
(setf (get ff_msgs::DockResult :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

# Result
int32 response
int32 CANCELLED                          = 5
int32 ALREADY_DOCKED                     = 4
int32 ALREADY_UNDOCKED                   = 3
int32 UNDOCKED                           = 2
int32 DOCKED                             = 1
int32 PREEMPTED                          = 0
int32 INVALID_COMMAND                    = -1
int32 INVALID_BERTH                      = -2
int32 NOT_IN_UNDOCKED_STATE              = -3
int32 NOT_IN_DOCKED_STATE                = -4
int32 SWITCH_TO_ML_FAILED                = -5
int32 SWITCH_TO_AR_FAILED                = -6
int32 SWITCH_TO_NO_FAILED                = -7
int32 PREP_DISABLE_FAILED                = -8
int32 PREP_ENABLE_FAILED                 = -9
int32 MOTION_APPROACH_FAILED             = -10
int32 MOTION_COMPLETE_FAILED             = -11
int32 MOTION_ATTACHED_FAILED             = -12
int32 EPS_UNDOCK_FAILED                  = -13
int32 EPS_DOCK_FAILED                    = -14
int32 TOO_FAR_AWAY_FROM_APPROACH         = -15

# Human readable FSM result for debugging
string fsm_result


")



(provide :ff_msgs/DockResult "0cc69ac3a301c7996578d2ee3e9b92a6")


