
(cl:in-package :asdf)

(defsystem "state_estimation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AircraftStateCalibMsg" :depends-on ("_package_AircraftStateCalibMsg"))
    (:file "_package_AircraftStateCalibMsg" :depends-on ("_package"))
    (:file "AircraftStateMsg" :depends-on ("_package_AircraftStateMsg"))
    (:file "_package_AircraftStateMsg" :depends-on ("_package"))
  ))