; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude StateDebug.msg.html

(cl:defclass <StateDebug> (roslisp-msg-protocol:ros-message)
  ((attitude
    :reader attitude
    :initarg :attitude
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (attitude_desired
    :reader attitude_desired
    :initarg :attitude_desired
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass StateDebug (<StateDebug>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StateDebug>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StateDebug)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<StateDebug> is deprecated: use quadrotor_msgs-msg:StateDebug instead.")))

(cl:ensure-generic-function 'attitude-val :lambda-list '(m))
(cl:defmethod attitude-val ((m <StateDebug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:attitude-val is deprecated.  Use quadrotor_msgs-msg:attitude instead.")
  (attitude m))

(cl:ensure-generic-function 'attitude_desired-val :lambda-list '(m))
(cl:defmethod attitude_desired-val ((m <StateDebug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:attitude_desired-val is deprecated.  Use quadrotor_msgs-msg:attitude_desired instead.")
  (attitude_desired m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StateDebug>) ostream)
  "Serializes a message object of type '<StateDebug>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'attitude) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'attitude_desired) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StateDebug>) istream)
  "Deserializes a message object of type '<StateDebug>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'attitude) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'attitude_desired) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StateDebug>)))
  "Returns string type for a message object of type '<StateDebug>"
  "quadrotor_msgs/StateDebug")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StateDebug)))
  "Returns string type for a message object of type 'StateDebug"
  "quadrotor_msgs/StateDebug")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StateDebug>)))
  "Returns md5sum for a message object of type '<StateDebug>"
  "4036a591967e8dadfb66a9590c8f625e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StateDebug)))
  "Returns md5sum for a message object of type 'StateDebug"
  "4036a591967e8dadfb66a9590c8f625e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StateDebug>)))
  "Returns full string definition for message of type '<StateDebug>"
  (cl:format cl:nil "geometry_msgs/Vector3 attitude~%geometry_msgs/Vector3 attitude_desired~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StateDebug)))
  "Returns full string definition for message of type 'StateDebug"
  (cl:format cl:nil "geometry_msgs/Vector3 attitude~%geometry_msgs/Vector3 attitude_desired~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StateDebug>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'attitude))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'attitude_desired))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StateDebug>))
  "Converts a ROS message object to a list"
  (cl:list 'StateDebug
    (cl:cons ':attitude (attitude msg))
    (cl:cons ':attitude_desired (attitude_desired msg))
))
