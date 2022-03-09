
(cl:in-package :asdf)

(defsystem "quadrotor_input-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :nav_msgs-msg
)
  :components ((:file "_package")
    (:file "SelectStateSource" :depends-on ("_package_SelectStateSource"))
    (:file "_package_SelectStateSource" :depends-on ("_package"))
    (:file "NotifyController" :depends-on ("_package_NotifyController"))
    (:file "_package_NotifyController" :depends-on ("_package"))
    (:file "CommandController" :depends-on ("_package_CommandController"))
    (:file "_package_CommandController" :depends-on ("_package"))
  ))