
(cl:in-package :asdf)

(defsystem "hunter_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "HunterMotorState" :depends-on ("_package_HunterMotorState"))
    (:file "_package_HunterMotorState" :depends-on ("_package"))
    (:file "HunterStatus" :depends-on ("_package_HunterStatus"))
    (:file "_package_HunterStatus" :depends-on ("_package"))
  ))