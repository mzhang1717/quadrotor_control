; Auto-generated. Do not edit!


(cl:in-package quadrotor_input-srv)


;//! \htmlinclude CommandController-request.msg.html

(cl:defclass <CommandController-request> (roslisp-msg-protocol:ros-message)
  ((running
    :reader running
    :initarg :running
    :type cl:boolean
    :initform cl:nil)
   (path
    :reader path
    :initarg :path
    :type nav_msgs-msg:Path
    :initform (cl:make-instance 'nav_msgs-msg:Path))
   (gains
    :reader gains
    :initarg :gains
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass CommandController-request (<CommandController-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CommandController-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CommandController-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_input-srv:<CommandController-request> is deprecated: use quadrotor_input-srv:CommandController-request instead.")))

(cl:ensure-generic-function 'running-val :lambda-list '(m))
(cl:defmethod running-val ((m <CommandController-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_input-srv:running-val is deprecated.  Use quadrotor_input-srv:running instead.")
  (running m))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <CommandController-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_input-srv:path-val is deprecated.  Use quadrotor_input-srv:path instead.")
  (path m))

(cl:ensure-generic-function 'gains-val :lambda-list '(m))
(cl:defmethod gains-val ((m <CommandController-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_input-srv:gains-val is deprecated.  Use quadrotor_input-srv:gains instead.")
  (gains m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CommandController-request>) ostream)
  "Serializes a message object of type '<CommandController-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'running) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'path) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'gains))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'gains))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CommandController-request>) istream)
  "Deserializes a message object of type '<CommandController-request>"
    (cl:setf (cl:slot-value msg 'running) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'path) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'gains) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'gains)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CommandController-request>)))
  "Returns string type for a service object of type '<CommandController-request>"
  "quadrotor_input/CommandControllerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommandController-request)))
  "Returns string type for a service object of type 'CommandController-request"
  "quadrotor_input/CommandControllerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CommandController-request>)))
  "Returns md5sum for a message object of type '<CommandController-request>"
  "f9a093ac383daf30a9e686870d8c6fc4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CommandController-request)))
  "Returns md5sum for a message object of type 'CommandController-request"
  "f9a093ac383daf30a9e686870d8c6fc4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CommandController-request>)))
  "Returns full string definition for message of type '<CommandController-request>"
  (cl:format cl:nil "bool running~%nav_msgs/Path path~%float64[] gains~%~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CommandController-request)))
  "Returns full string definition for message of type 'CommandController-request"
  (cl:format cl:nil "bool running~%nav_msgs/Path path~%float64[] gains~%~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CommandController-request>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'path))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'gains) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CommandController-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CommandController-request
    (cl:cons ':running (running msg))
    (cl:cons ':path (path msg))
    (cl:cons ':gains (gains msg))
))
;//! \htmlinclude CommandController-response.msg.html

(cl:defclass <CommandController-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass CommandController-response (<CommandController-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CommandController-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CommandController-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_input-srv:<CommandController-response> is deprecated: use quadrotor_input-srv:CommandController-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <CommandController-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_input-srv:success-val is deprecated.  Use quadrotor_input-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CommandController-response>) ostream)
  "Serializes a message object of type '<CommandController-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CommandController-response>) istream)
  "Deserializes a message object of type '<CommandController-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CommandController-response>)))
  "Returns string type for a service object of type '<CommandController-response>"
  "quadrotor_input/CommandControllerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommandController-response)))
  "Returns string type for a service object of type 'CommandController-response"
  "quadrotor_input/CommandControllerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CommandController-response>)))
  "Returns md5sum for a message object of type '<CommandController-response>"
  "f9a093ac383daf30a9e686870d8c6fc4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CommandController-response)))
  "Returns md5sum for a message object of type 'CommandController-response"
  "f9a093ac383daf30a9e686870d8c6fc4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CommandController-response>)))
  "Returns full string definition for message of type '<CommandController-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CommandController-response)))
  "Returns full string definition for message of type 'CommandController-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CommandController-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CommandController-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CommandController-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CommandController)))
  'CommandController-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CommandController)))
  'CommandController-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommandController)))
  "Returns string type for a service object of type '<CommandController>"
  "quadrotor_input/CommandController")