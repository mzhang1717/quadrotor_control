; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude ControllerError.msg.html

(cl:defclass <ControllerError> (roslisp-msg-protocol:ros-message)
  ((error
    :reader error
    :initarg :error
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (derivative
    :reader derivative
    :initarg :derivative
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass ControllerError (<ControllerError>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerError>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerError)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<ControllerError> is deprecated: use quadrotor_msgs-msg:ControllerError instead.")))

(cl:ensure-generic-function 'error-val :lambda-list '(m))
(cl:defmethod error-val ((m <ControllerError>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:error-val is deprecated.  Use quadrotor_msgs-msg:error instead.")
  (error m))

(cl:ensure-generic-function 'derivative-val :lambda-list '(m))
(cl:defmethod derivative-val ((m <ControllerError>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:derivative-val is deprecated.  Use quadrotor_msgs-msg:derivative instead.")
  (derivative m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerError>) ostream)
  "Serializes a message object of type '<ControllerError>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'error) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'derivative) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerError>) istream)
  "Deserializes a message object of type '<ControllerError>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'error) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'derivative) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerError>)))
  "Returns string type for a message object of type '<ControllerError>"
  "quadrotor_msgs/ControllerError")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerError)))
  "Returns string type for a message object of type 'ControllerError"
  "quadrotor_msgs/ControllerError")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerError>)))
  "Returns md5sum for a message object of type '<ControllerError>"
  "9908a00929a52cd30ca2a8fb177726b1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerError)))
  "Returns md5sum for a message object of type 'ControllerError"
  "9908a00929a52cd30ca2a8fb177726b1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerError>)))
  "Returns full string definition for message of type '<ControllerError>"
  (cl:format cl:nil "geometry_msgs/Vector3 error~%geometry_msgs/Vector3 derivative~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerError)))
  "Returns full string definition for message of type 'ControllerError"
  (cl:format cl:nil "geometry_msgs/Vector3 error~%geometry_msgs/Vector3 derivative~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerError>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'error))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'derivative))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerError>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerError
    (cl:cons ':error (error msg))
    (cl:cons ':derivative (derivative msg))
))
