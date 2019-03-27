
(cl:in-package :asdf)

(defsystem "jason_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Action" :depends-on ("_package_Action"))
    (:file "_package_Action" :depends-on ("_package"))
    (:file "ActionStatus" :depends-on ("_package_ActionStatus"))
    (:file "_package_ActionStatus" :depends-on ("_package"))
    (:file "Perception" :depends-on ("_package_Perception"))
    (:file "_package_Perception" :depends-on ("_package"))
  ))