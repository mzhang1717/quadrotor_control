; Auto-generated. Do not edit!


(cl:in-package quadrotor_input-srv)


;//! \htmlinclude NotifyController-request.msg.html

(cl:defclass <NotifyController-request> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass NotifyController-request (<NotifyController-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NotifyController-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NotifyController-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_input-srv:<NotifyController-request> is deprecated: use quadrotor_input-srv:NotifyController-request instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <NotifyController-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_input-srv:status-val is deprecated.  Use quadrotor_input-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<NotifyController-request>)))
    "Constants for message type '<NotifyController-request>"
  '((:INACTIVE . 0)
    (:ACTIVE . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'NotifyController-request)))
    "Constants for message type 'NotifyController-request"
  '((:INACTIVE . 0)
    (:ACTIVE . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NotifyController-request>) ostream)
  "Serializes a message object of type '<NotifyController-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NotifyController-request>) istream)
  "Deserializes a message object of type '<NotifyController-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NotifyController-request>)))
  "Returns string type for a service object of type '<NotifyController-request>"
  "quadrotor_input/NotifyControllerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NotifyController-request)))
  "Returns string type for a service object of type 'NotifyController-request"
  "quadrotor_input/NotifyControllerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NotifyController-request>)))
  "Returns md5sum for a message object of type '<NotifyController-request>"
  "7abd43fb91320bb7554c448c6f28715c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NotifyController-request)))
  "Returns md5sum for a message object of type 'NotifyController-request"
  "7abd43fb91320bb7554c448c6f28715c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NotifyController-request>)))
  "Returns full string definition for message of type '<NotifyController-request>"
  (cl:format cl:nil "uint8 status~%uint8 INACTIVE=0~%uint8 ACTIVE = 1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NotifyController-request)))
  "Returns full string definition for message of type 'NotifyController-request"
  (cl:format cl:nil "uint8 status~%uint8 INACTIVE=0~%uint8 ACTIVE = 1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NotifyController-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NotifyController-request>))
  "Converts a ROS message object to a list"
  (cl:list 'NotifyController-request
    (cl:cons ':status (status msg))
))
;//! \htmlinclude NotifyController-response.msg.html

(cl:defclass <NotifyController-response> (roslisp-msg-protocol:ros-message)
  ((controllerStatus
    :reader controllerStatus
    :initarg :controllerStatus
    :type cl:fixnum
    :initform 0))
)

(cl:defclass NotifyController-response (<NotifyController-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NotifyController-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NotifyController-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_input-srv:<NotifyController-response> is deprecated: use quadrotor_input-srv:NotifyController-response instead.")))

(cl:ensure-generic-function 'controllerStatus-val :lambda-list '(m))
(cl:defmethod controllerStatus-val ((m <NotifyController-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_input-srv:controllerStatus-val is deprecated.  Use quadrotor_input-srv:controllerStatus instead.")
  (controllerStatus m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<NotifyController-response>)))
    "Constants for message type '<NotifyController-response>"
  '((:INACTIVE . 0)
    (:ACTIVE . 1)
    (:ERROR . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'NotifyController-response)))
    "Constants for message type 'NotifyController-response"
  '((:INACTIVE . 0)
    (:ACTIVE . 1)
    (:ERROR . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NotifyController-response>) ostream)
  "Serializes a message object of type '<NotifyController-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'controllerStatus)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NotifyController-response>) istream)
  "Deserializes a message object of type '<NotifyController-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'controllerStatus)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NotifyController-response>)))
  "Returns string type for a service object of type '<NotifyController-response>"
  "quadrotor_input/NotifyControllerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NotifyController-response)))
  "Returns string type for a service object of type 'NotifyController-response"
  "quadrotor_input/NotifyControllerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NotifyController-response>)))
  "Returns md5sum for a message object of type '<NotifyController-response>"
  "7abd43fb91320bb7554c448c6f28715c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NotifyController-response)))
  "Returns md5sum for a message object of type 'NotifyController-response"
  "7abd43fb91320bb7554c448c6f28715c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NotifyController-response>)))
  "Returns full string definition for message of type '<NotifyController-response>"
  (cl:format cl:nil "uint8 controllerStatus~%uint8 INACTIVE=0~%uint8 ACTIVE = 1~%uint8 ERROR = 2~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NotifyController-response)))
  "Returns full string definition for message of type 'NotifyController-response"
  (cl:format cl:nil "uint8 controllerStatus~%uint8 INACTIVE=0~%uint8 ACTIVE = 1~%uint8 ERROR = 2~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NotifyController-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NotifyController-response>))
  "Converts a ROS message object to a list"
  (cl:list 'NotifyController-response
    (cl:cons ':controllerStatus (controllerStatus msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'NotifyController)))
  'NotifyController-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'NotifyController)))
  'NotifyController-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NotifyController)))
  "Returns string type for a service object of type '<NotifyController>"
  "quadrotor_input/NotifyController")