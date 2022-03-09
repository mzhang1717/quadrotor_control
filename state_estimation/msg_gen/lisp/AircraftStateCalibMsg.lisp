; Auto-generated. Do not edit!


(cl:in-package state_estimation-msg)


;//! \htmlinclude AircraftStateCalibMsg.msg.html

(cl:defclass <AircraftStateCalibMsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (rotation
    :reader rotation
    :initarg :rotation
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (velocity
    :reader velocity
    :initarg :velocity
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (accel_bias
    :reader accel_bias
    :initarg :accel_bias
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (gyro_bias
    :reader gyro_bias
    :initarg :gyro_bias
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (pos_c
    :reader pos_c
    :initarg :pos_c
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (rot_c
    :reader rot_c
    :initarg :rot_c
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (pos_v
    :reader pos_v
    :initarg :pos_v
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (rot_v
    :reader rot_v
    :initarg :rot_v
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (covariance
    :reader covariance
    :initarg :covariance
    :type (cl:vector cl:float)
   :initform (cl:make-array 729 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass AircraftStateCalibMsg (<AircraftStateCalibMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AircraftStateCalibMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AircraftStateCalibMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimation-msg:<AircraftStateCalibMsg> is deprecated: use state_estimation-msg:AircraftStateCalibMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AircraftStateCalibMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimation-msg:header-val is deprecated.  Use state_estimation-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <AircraftStateCalibMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimation-msg:position-val is deprecated.  Use state_estimation-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'rotation-val :lambda-list '(m))
(cl:defmethod rotation-val ((m <AircraftStateCalibMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimation-msg:rotation-val is deprecated.  Use state_estimation-msg:rotation instead.")
  (rotation m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <AircraftStateCalibMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimation-msg:velocity-val is deprecated.  Use state_estimation-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'accel_bias-val :lambda-list '(m))
(cl:defmethod accel_bias-val ((m <AircraftStateCalibMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimation-msg:accel_bias-val is deprecated.  Use state_estimation-msg:accel_bias instead.")
  (accel_bias m))

(cl:ensure-generic-function 'gyro_bias-val :lambda-list '(m))
(cl:defmethod gyro_bias-val ((m <AircraftStateCalibMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimation-msg:gyro_bias-val is deprecated.  Use state_estimation-msg:gyro_bias instead.")
  (gyro_bias m))

(cl:ensure-generic-function 'pos_c-val :lambda-list '(m))
(cl:defmethod pos_c-val ((m <AircraftStateCalibMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimation-msg:pos_c-val is deprecated.  Use state_estimation-msg:pos_c instead.")
  (pos_c m))

(cl:ensure-generic-function 'rot_c-val :lambda-list '(m))
(cl:defmethod rot_c-val ((m <AircraftStateCalibMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimation-msg:rot_c-val is deprecated.  Use state_estimation-msg:rot_c instead.")
  (rot_c m))

(cl:ensure-generic-function 'pos_v-val :lambda-list '(m))
(cl:defmethod pos_v-val ((m <AircraftStateCalibMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimation-msg:pos_v-val is deprecated.  Use state_estimation-msg:pos_v instead.")
  (pos_v m))

(cl:ensure-generic-function 'rot_v-val :lambda-list '(m))
(cl:defmethod rot_v-val ((m <AircraftStateCalibMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimation-msg:rot_v-val is deprecated.  Use state_estimation-msg:rot_v instead.")
  (rot_v m))

(cl:ensure-generic-function 'covariance-val :lambda-list '(m))
(cl:defmethod covariance-val ((m <AircraftStateCalibMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimation-msg:covariance-val is deprecated.  Use state_estimation-msg:covariance instead.")
  (covariance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AircraftStateCalibMsg>) ostream)
  "Serializes a message object of type '<AircraftStateCalibMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'rotation) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'accel_bias) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'gyro_bias) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pos_c) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'rot_c) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pos_v) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'rot_v) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'covariance))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AircraftStateCalibMsg>) istream)
  "Deserializes a message object of type '<AircraftStateCalibMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'rotation) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'accel_bias) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'gyro_bias) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pos_c) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'rot_c) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pos_v) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'rot_v) istream)
  (cl:setf (cl:slot-value msg 'covariance) (cl:make-array 729))
  (cl:let ((vals (cl:slot-value msg 'covariance)))
    (cl:dotimes (i 729)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AircraftStateCalibMsg>)))
  "Returns string type for a message object of type '<AircraftStateCalibMsg>"
  "state_estimation/AircraftStateCalibMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AircraftStateCalibMsg)))
  "Returns string type for a message object of type 'AircraftStateCalibMsg"
  "state_estimation/AircraftStateCalibMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AircraftStateCalibMsg>)))
  "Returns md5sum for a message object of type '<AircraftStateCalibMsg>"
  "391e803414d8347989ae2ed97dfcdad3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AircraftStateCalibMsg)))
  "Returns md5sum for a message object of type 'AircraftStateCalibMsg"
  "391e803414d8347989ae2ed97dfcdad3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AircraftStateCalibMsg>)))
  "Returns full string definition for message of type '<AircraftStateCalibMsg>"
  (cl:format cl:nil "Header header~%geometry_msgs/Point position~%geometry_msgs/Quaternion rotation~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 accel_bias~%geometry_msgs/Vector3 gyro_bias~%geometry_msgs/Point pos_c~%geometry_msgs/Quaternion rot_c~%geometry_msgs/Point pos_v~%geometry_msgs/Quaternion rot_v~%float64[729] covariance~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AircraftStateCalibMsg)))
  "Returns full string definition for message of type 'AircraftStateCalibMsg"
  (cl:format cl:nil "Header header~%geometry_msgs/Point position~%geometry_msgs/Quaternion rotation~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 accel_bias~%geometry_msgs/Vector3 gyro_bias~%geometry_msgs/Point pos_c~%geometry_msgs/Quaternion rot_c~%geometry_msgs/Point pos_v~%geometry_msgs/Quaternion rot_v~%float64[729] covariance~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AircraftStateCalibMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'rotation))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'accel_bias))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'gyro_bias))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pos_c))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'rot_c))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pos_v))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'rot_v))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'covariance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AircraftStateCalibMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'AircraftStateCalibMsg
    (cl:cons ':header (header msg))
    (cl:cons ':position (position msg))
    (cl:cons ':rotation (rotation msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':accel_bias (accel_bias msg))
    (cl:cons ':gyro_bias (gyro_bias msg))
    (cl:cons ':pos_c (pos_c msg))
    (cl:cons ':rot_c (rot_c msg))
    (cl:cons ':pos_v (pos_v msg))
    (cl:cons ':rot_v (rot_v msg))
    (cl:cons ':covariance (covariance msg))
))
