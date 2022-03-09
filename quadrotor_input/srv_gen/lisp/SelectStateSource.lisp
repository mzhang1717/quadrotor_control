; Auto-generated. Do not edit!


(cl:in-package quadrotor_input-srv)


;//! \htmlinclude SelectStateSource-request.msg.html

(cl:defclass <SelectStateSource-request> (roslisp-msg-protocol:ros-message)
  ((strPoseTopicSelected
    :reader strPoseTopicSelected
    :initarg :strPoseTopicSelected
    :type cl:string
    :initform "")
   (strVelocityTopicSelected
    :reader strVelocityTopicSelected
    :initarg :strVelocityTopicSelected
    :type cl:string
    :initform ""))
)

(cl:defclass SelectStateSource-request (<SelectStateSource-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SelectStateSource-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SelectStateSource-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_input-srv:<SelectStateSource-request> is deprecated: use quadrotor_input-srv:SelectStateSource-request instead.")))

(cl:ensure-generic-function 'strPoseTopicSelected-val :lambda-list '(m))
(cl:defmethod strPoseTopicSelected-val ((m <SelectStateSource-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_input-srv:strPoseTopicSelected-val is deprecated.  Use quadrotor_input-srv:strPoseTopicSelected instead.")
  (strPoseTopicSelected m))

(cl:ensure-generic-function 'strVelocityTopicSelected-val :lambda-list '(m))
(cl:defmethod strVelocityTopicSelected-val ((m <SelectStateSource-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_input-srv:strVelocityTopicSelected-val is deprecated.  Use quadrotor_input-srv:strVelocityTopicSelected instead.")
  (strVelocityTopicSelected m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SelectStateSource-request>) ostream)
  "Serializes a message object of type '<SelectStateSource-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'strPoseTopicSelected))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'strPoseTopicSelected))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'strVelocityTopicSelected))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'strVelocityTopicSelected))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SelectStateSource-request>) istream)
  "Deserializes a message object of type '<SelectStateSource-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'strPoseTopicSelected) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'strPoseTopicSelected) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'strVelocityTopicSelected) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'strVelocityTopicSelected) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SelectStateSource-request>)))
  "Returns string type for a service object of type '<SelectStateSource-request>"
  "quadrotor_input/SelectStateSourceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SelectStateSource-request)))
  "Returns string type for a service object of type 'SelectStateSource-request"
  "quadrotor_input/SelectStateSourceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SelectStateSource-request>)))
  "Returns md5sum for a message object of type '<SelectStateSource-request>"
  "11317bd1430c644f8910ccd389a7161d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SelectStateSource-request)))
  "Returns md5sum for a message object of type 'SelectStateSource-request"
  "11317bd1430c644f8910ccd389a7161d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SelectStateSource-request>)))
  "Returns full string definition for message of type '<SelectStateSource-request>"
  (cl:format cl:nil "string strPoseTopicSelected~%string strVelocityTopicSelected~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SelectStateSource-request)))
  "Returns full string definition for message of type 'SelectStateSource-request"
  (cl:format cl:nil "string strPoseTopicSelected~%string strVelocityTopicSelected~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SelectStateSource-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'strPoseTopicSelected))
     4 (cl:length (cl:slot-value msg 'strVelocityTopicSelected))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SelectStateSource-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SelectStateSource-request
    (cl:cons ':strPoseTopicSelected (strPoseTopicSelected msg))
    (cl:cons ':strVelocityTopicSelected (strVelocityTopicSelected msg))
))
;//! \htmlinclude SelectStateSource-response.msg.html

(cl:defclass <SelectStateSource-response> (roslisp-msg-protocol:ros-message)
  ((bSuccess
    :reader bSuccess
    :initarg :bSuccess
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SelectStateSource-response (<SelectStateSource-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SelectStateSource-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SelectStateSource-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_input-srv:<SelectStateSource-response> is deprecated: use quadrotor_input-srv:SelectStateSource-response instead.")))

(cl:ensure-generic-function 'bSuccess-val :lambda-list '(m))
(cl:defmethod bSuccess-val ((m <SelectStateSource-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_input-srv:bSuccess-val is deprecated.  Use quadrotor_input-srv:bSuccess instead.")
  (bSuccess m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SelectStateSource-response>) ostream)
  "Serializes a message object of type '<SelectStateSource-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'bSuccess) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SelectStateSource-response>) istream)
  "Deserializes a message object of type '<SelectStateSource-response>"
    (cl:setf (cl:slot-value msg 'bSuccess) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SelectStateSource-response>)))
  "Returns string type for a service object of type '<SelectStateSource-response>"
  "quadrotor_input/SelectStateSourceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SelectStateSource-response)))
  "Returns string type for a service object of type 'SelectStateSource-response"
  "quadrotor_input/SelectStateSourceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SelectStateSource-response>)))
  "Returns md5sum for a message object of type '<SelectStateSource-response>"
  "11317bd1430c644f8910ccd389a7161d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SelectStateSource-response)))
  "Returns md5sum for a message object of type 'SelectStateSource-response"
  "11317bd1430c644f8910ccd389a7161d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SelectStateSource-response>)))
  "Returns full string definition for message of type '<SelectStateSource-response>"
  (cl:format cl:nil "bool bSuccess~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SelectStateSource-response)))
  "Returns full string definition for message of type 'SelectStateSource-response"
  (cl:format cl:nil "bool bSuccess~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SelectStateSource-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SelectStateSource-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SelectStateSource-response
    (cl:cons ':bSuccess (bSuccess msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SelectStateSource)))
  'SelectStateSource-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SelectStateSource)))
  'SelectStateSource-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SelectStateSource)))
  "Returns string type for a service object of type '<SelectStateSource>"
  "quadrotor_input/SelectStateSource")