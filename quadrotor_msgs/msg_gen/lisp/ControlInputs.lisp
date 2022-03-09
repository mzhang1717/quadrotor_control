; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude ControlInputs.msg.html

(cl:defclass <ControlInputs> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (scaledInputs
    :reader scaledInputs
    :initarg :scaledInputs
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 16 :element-type 'cl:fixnum :initial-element 0))
   (zigbeeInputs
    :reader zigbeeInputs
    :initarg :zigbeeInputs
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 16 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass ControlInputs (<ControlInputs>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlInputs>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlInputs)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<ControlInputs> is deprecated: use quadrotor_msgs-msg:ControlInputs instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ControlInputs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:header-val is deprecated.  Use quadrotor_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'scaledInputs-val :lambda-list '(m))
(cl:defmethod scaledInputs-val ((m <ControlInputs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:scaledInputs-val is deprecated.  Use quadrotor_msgs-msg:scaledInputs instead.")
  (scaledInputs m))

(cl:ensure-generic-function 'zigbeeInputs-val :lambda-list '(m))
(cl:defmethod zigbeeInputs-val ((m <ControlInputs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:zigbeeInputs-val is deprecated.  Use quadrotor_msgs-msg:zigbeeInputs instead.")
  (zigbeeInputs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlInputs>) ostream)
  "Serializes a message object of type '<ControlInputs>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'scaledInputs))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'zigbeeInputs))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlInputs>) istream)
  "Deserializes a message object of type '<ControlInputs>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'scaledInputs) (cl:make-array 16))
  (cl:let ((vals (cl:slot-value msg 'scaledInputs)))
    (cl:dotimes (i 16)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  (cl:setf (cl:slot-value msg 'zigbeeInputs) (cl:make-array 16))
  (cl:let ((vals (cl:slot-value msg 'zigbeeInputs)))
    (cl:dotimes (i 16)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlInputs>)))
  "Returns string type for a message object of type '<ControlInputs>"
  "quadrotor_msgs/ControlInputs")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlInputs)))
  "Returns string type for a message object of type 'ControlInputs"
  "quadrotor_msgs/ControlInputs")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlInputs>)))
  "Returns md5sum for a message object of type '<ControlInputs>"
  "37612a0241cc186045688e2e42377320")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlInputs)))
  "Returns md5sum for a message object of type 'ControlInputs"
  "37612a0241cc186045688e2e42377320")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlInputs>)))
  "Returns full string definition for message of type '<ControlInputs>"
  (cl:format cl:nil "Header header~%int16[16] scaledInputs~%int16[16] zigbeeInputs~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlInputs)))
  "Returns full string definition for message of type 'ControlInputs"
  (cl:format cl:nil "Header header~%int16[16] scaledInputs~%int16[16] zigbeeInputs~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlInputs>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'scaledInputs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'zigbeeInputs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlInputs>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlInputs
    (cl:cons ':header (header msg))
    (cl:cons ':scaledInputs (scaledInputs msg))
    (cl:cons ':zigbeeInputs (zigbeeInputs msg))
))
