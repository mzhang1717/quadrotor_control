; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude ControlWarn.msg.html

(cl:defclass <ControlWarn> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (warnName
    :reader warnName
    :initarg :warnName
    :type cl:string
    :initform "")
   (warnInfo
    :reader warnInfo
    :initarg :warnInfo
    :type cl:float
    :initform 0.0))
)

(cl:defclass ControlWarn (<ControlWarn>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlWarn>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlWarn)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<ControlWarn> is deprecated: use quadrotor_msgs-msg:ControlWarn instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ControlWarn>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:header-val is deprecated.  Use quadrotor_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'warnName-val :lambda-list '(m))
(cl:defmethod warnName-val ((m <ControlWarn>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:warnName-val is deprecated.  Use quadrotor_msgs-msg:warnName instead.")
  (warnName m))

(cl:ensure-generic-function 'warnInfo-val :lambda-list '(m))
(cl:defmethod warnInfo-val ((m <ControlWarn>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:warnInfo-val is deprecated.  Use quadrotor_msgs-msg:warnInfo instead.")
  (warnInfo m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlWarn>) ostream)
  "Serializes a message object of type '<ControlWarn>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'warnName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'warnName))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'warnInfo))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlWarn>) istream)
  "Deserializes a message object of type '<ControlWarn>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'warnName) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'warnName) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'warnInfo) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlWarn>)))
  "Returns string type for a message object of type '<ControlWarn>"
  "quadrotor_msgs/ControlWarn")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlWarn)))
  "Returns string type for a message object of type 'ControlWarn"
  "quadrotor_msgs/ControlWarn")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlWarn>)))
  "Returns md5sum for a message object of type '<ControlWarn>"
  "c5676241895812ed60a907c0d50bb0fd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlWarn)))
  "Returns md5sum for a message object of type 'ControlWarn"
  "c5676241895812ed60a907c0d50bb0fd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlWarn>)))
  "Returns full string definition for message of type '<ControlWarn>"
  (cl:format cl:nil "std_msgs/Header header~%string warnName~%float64 warnInfo~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlWarn)))
  "Returns full string definition for message of type 'ControlWarn"
  (cl:format cl:nil "std_msgs/Header header~%string warnName~%float64 warnInfo~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlWarn>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'warnName))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlWarn>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlWarn
    (cl:cons ':header (header msg))
    (cl:cons ':warnName (warnName msg))
    (cl:cons ':warnInfo (warnInfo msg))
))
