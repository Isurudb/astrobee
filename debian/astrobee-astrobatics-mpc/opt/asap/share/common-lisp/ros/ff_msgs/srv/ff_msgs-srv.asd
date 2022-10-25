
(cl:in-package :asdf)

(defsystem "ff_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :ff_msgs-msg
               :geometry_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "ConfigureCamera" :depends-on ("_package_ConfigureCamera"))
    (:file "_package_ConfigureCamera" :depends-on ("_package"))
    (:file "EnableCamera" :depends-on ("_package_EnableCamera"))
    (:file "_package_EnableCamera" :depends-on ("_package"))
    (:file "EnableRecording" :depends-on ("_package_EnableRecording"))
    (:file "_package_EnableRecording" :depends-on ("_package"))
    (:file "GetFloat" :depends-on ("_package_GetFloat"))
    (:file "_package_GetFloat" :depends-on ("_package"))
    (:file "GetMap" :depends-on ("_package_GetMap"))
    (:file "_package_GetMap" :depends-on ("_package"))
    (:file "GetOccupancyMap" :depends-on ("_package_GetOccupancyMap"))
    (:file "_package_GetOccupancyMap" :depends-on ("_package"))
    (:file "GetPipelines" :depends-on ("_package_GetPipelines"))
    (:file "_package_GetPipelines" :depends-on ("_package"))
    (:file "GetZones" :depends-on ("_package_GetZones"))
    (:file "_package_GetZones" :depends-on ("_package"))
    (:file "RegisterPlanner" :depends-on ("_package_RegisterPlanner"))
    (:file "_package_RegisterPlanner" :depends-on ("_package"))
    (:file "ResetMap" :depends-on ("_package_ResetMap"))
    (:file "_package_ResetMap" :depends-on ("_package"))
    (:file "SetBool" :depends-on ("_package_SetBool"))
    (:file "_package_SetBool" :depends-on ("_package"))
    (:file "SetDataToDisk" :depends-on ("_package_SetDataToDisk"))
    (:file "_package_SetDataToDisk" :depends-on ("_package"))
    (:file "SetEkfInput" :depends-on ("_package_SetEkfInput"))
    (:file "_package_SetEkfInput" :depends-on ("_package"))
    (:file "SetFloat" :depends-on ("_package_SetFloat"))
    (:file "_package_SetFloat" :depends-on ("_package"))
    (:file "SetInertia" :depends-on ("_package_SetInertia"))
    (:file "_package_SetInertia" :depends-on ("_package"))
    (:file "SetRate" :depends-on ("_package_SetRate"))
    (:file "_package_SetRate" :depends-on ("_package"))
    (:file "SetState" :depends-on ("_package_SetState"))
    (:file "_package_SetState" :depends-on ("_package"))
    (:file "SetStreamingLights" :depends-on ("_package_SetStreamingLights"))
    (:file "_package_SetStreamingLights" :depends-on ("_package"))
    (:file "SetZones" :depends-on ("_package_SetZones"))
    (:file "_package_SetZones" :depends-on ("_package"))
    (:file "Trigger" :depends-on ("_package_Trigger"))
    (:file "_package_Trigger" :depends-on ("_package"))
    (:file "UnloadLoadNodelet" :depends-on ("_package_UnloadLoadNodelet"))
    (:file "_package_UnloadLoadNodelet" :depends-on ("_package"))
    (:file "VisualeyezConfig" :depends-on ("_package_VisualeyezConfig"))
    (:file "_package_VisualeyezConfig" :depends-on ("_package"))
  ))