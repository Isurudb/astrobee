
(cl:in-package :asdf)

(defsystem "coordinator-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "StatusPrimary" :depends-on ("_package_StatusPrimary"))
    (:file "_package_StatusPrimary" :depends-on ("_package"))
    (:file "StatusSecondary" :depends-on ("_package_StatusSecondary"))
    (:file "_package_StatusSecondary" :depends-on ("_package"))
    (:file "TestNumber" :depends-on ("_package_TestNumber"))
    (:file "_package_TestNumber" :depends-on ("_package"))
  ))