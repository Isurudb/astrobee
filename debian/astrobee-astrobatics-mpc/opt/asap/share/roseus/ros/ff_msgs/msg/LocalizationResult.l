;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::LocalizationResult)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'LocalizationResult (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::LOCALIZATIONRESULT")
  (make-package "FF_MSGS::LOCALIZATIONRESULT"))

(in-package "ROS")
;;//! \htmlinclude LocalizationResult.msg.html


(intern "*PIPELINE_ALREADY_ACTIVE*" (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(shadow '*PIPELINE_ALREADY_ACTIVE* (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(defconstant ff_msgs::LocalizationResult::*PIPELINE_ALREADY_ACTIVE* 2)
(intern "*SUCCESS*" (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(shadow '*SUCCESS* (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(defconstant ff_msgs::LocalizationResult::*SUCCESS* 1)
(intern "*PREEMPTED*" (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(shadow '*PREEMPTED* (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(defconstant ff_msgs::LocalizationResult::*PREEMPTED* 0)
(intern "*CANCELLED*" (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(shadow '*CANCELLED* (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(defconstant ff_msgs::LocalizationResult::*CANCELLED* -1)
(intern "*INVALID_PIPELINE*" (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(shadow '*INVALID_PIPELINE* (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(defconstant ff_msgs::LocalizationResult::*INVALID_PIPELINE* -2)
(intern "*INVALID_COMMAND*" (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(shadow '*INVALID_COMMAND* (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(defconstant ff_msgs::LocalizationResult::*INVALID_COMMAND* -3)
(intern "*FILTER_NOT_IN_USE*" (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(shadow '*FILTER_NOT_IN_USE* (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(defconstant ff_msgs::LocalizationResult::*FILTER_NOT_IN_USE* -4)
(intern "*OPTICAL_FLOW_FAILED*" (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(shadow '*OPTICAL_FLOW_FAILED* (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(defconstant ff_msgs::LocalizationResult::*OPTICAL_FLOW_FAILED* -5)
(intern "*PIPELINE_TOGGLE_FAILED*" (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(shadow '*PIPELINE_TOGGLE_FAILED* (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(defconstant ff_msgs::LocalizationResult::*PIPELINE_TOGGLE_FAILED* -6)
(intern "*PIPELINE_USE_FAILED*" (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(shadow '*PIPELINE_USE_FAILED* (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(defconstant ff_msgs::LocalizationResult::*PIPELINE_USE_FAILED* -7)
(intern "*PIPELINE_UNSTABLE*" (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(shadow '*PIPELINE_UNSTABLE* (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(defconstant ff_msgs::LocalizationResult::*PIPELINE_UNSTABLE* -8)
(intern "*SET_INPUT_FAILED*" (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(shadow '*SET_INPUT_FAILED* (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(defconstant ff_msgs::LocalizationResult::*SET_INPUT_FAILED* -9)
(intern "*ESTIMATE_BIAS_FAILED*" (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(shadow '*ESTIMATE_BIAS_FAILED* (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(defconstant ff_msgs::LocalizationResult::*ESTIMATE_BIAS_FAILED* -10)
(intern "*RESET_FAILED*" (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(shadow '*RESET_FAILED* (find-package "FF_MSGS::LOCALIZATIONRESULT"))
(defconstant ff_msgs::LocalizationResult::*RESET_FAILED* -11)
(defclass ff_msgs::LocalizationResult
  :super ros::object
  :slots (_response _fsm_result ))

(defmethod ff_msgs::LocalizationResult
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

(setf (get ff_msgs::LocalizationResult :md5sum-) "309c1ead50fb170acfed9e9b67a66d27")
(setf (get ff_msgs::LocalizationResult :datatype-) "ff_msgs/LocalizationResult")
(setf (get ff_msgs::LocalizationResult :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

int32 response
int32 PIPELINE_ALREADY_ACTIVE     =  2   # We are already on this mode
int32 SUCCESS                     =  1   # The switch was successful
int32 PREEMPTED                   =  0   # Preempted by another action goal
int32 CANCELLED                   = -1   # We canceled our own request
int32 INVALID_PIPELINE            = -2   # Not a valid pipeline in command
int32 INVALID_COMMAND             = -3   # Not a valid command type
int32 FILTER_NOT_IN_USE           = -4   # Reset/bias requires filter
int32 OPTICAL_FLOW_FAILED         = -5   # Optical flow could not be toggled
int32 PIPELINE_TOGGLE_FAILED      = -6   # Pipeline could not be toggled
int32 PIPELINE_USE_FAILED         = -7   # Pipeline could not be used
int32 PIPELINE_UNSTABLE           = -8   # Pipeline did not go stable
int32 SET_INPUT_FAILED            = -9   # EKF could not be set to new mode
int32 ESTIMATE_BIAS_FAILED        = -10  # Estimate bias service call failed
int32 RESET_FAILED                = -11  # Reset service call failed

# Human readable FSM result for debugging
string fsm_result


")



(provide :ff_msgs/LocalizationResult "309c1ead50fb170acfed9e9b67a66d27")

