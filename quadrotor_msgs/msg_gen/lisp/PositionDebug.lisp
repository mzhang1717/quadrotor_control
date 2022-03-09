; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude PositionDebug.msg.html

(cl:defclass <PositionDebug> (roslisp-msg-protocol:ros-message)
  ((base_throttle
    :reader base_throttle
    :initarg :base_throttle
    :type cl:float
    :initform 0.0)
   (scale
    :reader scale
    :initarg :scale
    :type cl:float
    :initform 0.0)
   (voltage
    :reader voltage
    :initarg :voltage
    :type cl:float
    :initform 0.0)
   (xy_error
    :reader xy_error
    :initarg :xy_error
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (vx_error
    :reader vx_error
    :initarg :vx_error
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (vy_error
    :reader vy_error
    :initarg :vy_error
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (z_error
    :reader z_error
    :initarg :z_error
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (vz_error
    :reader vz_error
    :initarg :vz_error
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (yaw_error
    :reader yaw_error
    :initarg :yaw_error
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (desired_xyz_yaw
    :reader desired_xyz_yaw
    :initarg :desired_xyz_yaw
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (current_xyz_yaw
    :reader current_xyz_yaw
    :initarg :current_xyz_yaw
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (desired_velocity
    :reader desired_velocity
    :initarg :desired_velocity
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (desired_acceleration
    :reader desired_acceleration
    :initarg :desired_acceleration
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (cmd
    :reader cmd
    :initarg :cmd
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (cmd_limited
    :reader cmd_limited
    :initarg :cmd_limited
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion)))
)

(cl:defclass PositionDebug (<PositionDebug>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PositionDebug>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PositionDebug)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<PositionDebug> is deprecated: use quadrotor_msgs-msg:PositionDebug instead.")))

(cl:ensure-generic-function 'base_throttle-val :lambda-list '(m))
(cl:defmethod base_throttle-val ((m <PositionDebug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:base_throttle-val is deprecated.  Use quadrotor_msgs-msg:base_throttle instead.")
  (base_throttle m))

(cl:ensure-generic-function 'scale-val :lambda-list '(m))
(cl:defmethod scale-val ((m <PositionDebug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:scale-val is deprecated.  Use quadrotor_msgs-msg:scale instead.")
  (scale m))

(cl:ensure-generic-function 'voltage-val :lambda-list '(m))
(cl:defmethod voltage-val ((m <PositionDebug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:voltage-val is deprecated.  Use quadrotor_msgs-msg:voltage instead.")
  (voltage m))

(cl:ensure-generic-function 'xy_error-val :lambda-list '(m))
(cl:defmethod xy_error-val ((m <PositionDebug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:xy_error-val is deprecated.  Use quadrotor_msgs-msg:xy_error instead.")
  (xy_error m))

(cl:ensure-generic-function 'vx_error-val :lambda-list '(m))
(cl:defmethod vx_error-val ((m <PositionDebug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:vx_error-val is deprecated.  Use quadrotor_msgs-msg:vx_error instead.")
  (vx_error m))

(cl:ensure-generic-function 'vy_error-val :lambda-list '(m))
(cl:defmethod vy_error-val ((m <PositionDebug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:vy_error-val is deprecated.  Use quadrotor_msgs-msg:vy_error instead.")
  (vy_error m))

(cl:ensure-generic-function 'z_error-val :lambda-list '(m))
(cl:defmethod z_error-val ((m <PositionDebug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:z_error-val is deprecated.  Use quadrotor_msgs-msg:z_error instead.")
  (z_error m))

(cl:ensure-generic-function 'vz_error-val :lambda-list '(m))
(cl:defmethod vz_error-val ((m <PositionDebug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:vz_error-val is deprecated.  Use quadrotor_msgs-msg:vz_error instead.")
  (vz_error m))

(cl:ensure-generic-function 'yaw_error-val :lambda-list '(m))
(cl:defmethod yaw_error-val ((m <PositionDebug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:yaw_error-val is deprecated.  Use quadrotor_msgs-msg:yaw_error instead.")
  (yaw_error m))

(cl:ensure-generic-function 'desired_xyz_yaw-val :lambda-list '(m))
(cl:defmethod desired_xyz_yaw-val ((m <PositionDebug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:desired_xyz_yaw-val is deprecated.  Use quadrotor_msgs-msg:desired_xyz_yaw instead.")
  (desired_xyz_yaw m))

(cl:ensure-generic-function 'current_xyz_yaw-val :lambda-list '(m))
(cl:defmethod current_xyz_yaw-val ((m <PositionDebug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:current_xyz_yaw-val is deprecated.  Use quadrotor_msgs-msg:current_xyz_yaw instead.")
  (current_xyz_yaw m))

(cl:ensure-generic-function 'desired_velocity-val :lambda-list '(m))
(cl:defmethod desired_velocity-val ((m <PositionDebug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:desired_velocity-val is deprecated.  Use quadrotor_msgs-msg:desired_velocity instead.")
  (desired_velocity m))

(cl:ensure-generic-function 'desired_acceleration-val :lambda-list '(m))
(cl:defmethod desired_acceleration-val ((m <PositionDebug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:desired_acceleration-val is deprecated.  Use quadrotor_msgs-msg:desired_acceleration instead.")
  (desired_acceleration m))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <PositionDebug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:cmd-val is deprecated.  Use quadrotor_msgs-msg:cmd instead.")
  (cmd m))

(cl:ensure-generic-function 'cmd_limited-val :lambda-list '(m))
(cl:defmethod cmd_limited-val ((m <PositionDebug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:cmd_limited-val is deprecated.  Use quadrotor_msgs-msg:cmd_limited instead.")
  (cmd_limited m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PositionDebug>) ostream)
  "Serializes a message object of type '<PositionDebug>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'base_throttle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'scale))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'voltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'xy_error) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'vx_error) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'vy_error) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'z_error) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'vz_error) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'yaw_error) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'desired_xyz_yaw) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'current_xyz_yaw) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'desired_velocity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'desired_acceleration) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cmd) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cmd_limited) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PositionDebug>) istream)
  "Deserializes a message object of type '<PositionDebug>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'base_throttle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'scale) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'voltage) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'xy_error) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'vx_error) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'vy_error) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'z_error) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'vz_error) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'yaw_error) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'desired_xyz_yaw) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'current_xyz_yaw) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'desired_velocity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'desired_acceleration) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cmd) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cmd_limited) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PositionDebug>)))
  "Returns string type for a message object of type '<PositionDebug>"
  "quadrotor_msgs/PositionDebug")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PositionDebug)))
  "Returns string type for a message object of type 'PositionDebug"
  "quadrotor_msgs/PositionDebug")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PositionDebug>)))
  "Returns md5sum for a message object of type '<PositionDebug>"
  "93d1333b166d8ea6c1d79f7209332822")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PositionDebug)))
  "Returns md5sum for a message object of type 'PositionDebug"
  "93d1333b166d8ea6c1d79f7209332822")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PositionDebug>)))
  "Returns full string definition for message of type '<PositionDebug>"
  (cl:format cl:nil "float64 base_throttle~%float64 scale~%float64 voltage~%geometry_msgs/Vector3 xy_error~%geometry_msgs/Vector3 vx_error~%geometry_msgs/Vector3 vy_error~%geometry_msgs/Vector3 z_error~%geometry_msgs/Vector3 vz_error~%geometry_msgs/Vector3 yaw_error~%geometry_msgs/Quaternion desired_xyz_yaw~%geometry_msgs/Quaternion current_xyz_yaw~%geometry_msgs/Vector3 desired_velocity~%geometry_msgs/Vector3 desired_acceleration~%geometry_msgs/Quaternion cmd~%geometry_msgs/Quaternion cmd_limited~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PositionDebug)))
  "Returns full string definition for message of type 'PositionDebug"
  (cl:format cl:nil "float64 base_throttle~%float64 scale~%float64 voltage~%geometry_msgs/Vector3 xy_error~%geometry_msgs/Vector3 vx_error~%geometry_msgs/Vector3 vy_error~%geometry_msgs/Vector3 z_error~%geometry_msgs/Vector3 vz_error~%geometry_msgs/Vector3 yaw_error~%geometry_msgs/Quaternion desired_xyz_yaw~%geometry_msgs/Quaternion current_xyz_yaw~%geometry_msgs/Vector3 desired_velocity~%geometry_msgs/Vector3 desired_acceleration~%geometry_msgs/Quaternion cmd~%geometry_msgs/Quaternion cmd_limited~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PositionDebug>))
  (cl:+ 0
     8
     8
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'xy_error))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'vx_error))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'vy_error))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'z_error))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'vz_error))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'yaw_error))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'desired_xyz_yaw))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'current_xyz_yaw))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'desired_velocity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'desired_acceleration))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cmd))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cmd_limited))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PositionDebug>))
  "Converts a ROS message object to a list"
  (cl:list 'PositionDebug
    (cl:cons ':base_throttle (base_throttle msg))
    (cl:cons ':scale (scale msg))
    (cl:cons ':voltage (voltage msg))
    (cl:cons ':xy_error (xy_error msg))
    (cl:cons ':vx_error (vx_error msg))
    (cl:cons ':vy_error (vy_error msg))
    (cl:cons ':z_error (z_error msg))
    (cl:cons ':vz_error (vz_error msg))
    (cl:cons ':yaw_error (yaw_error msg))
    (cl:cons ':desired_xyz_yaw (desired_xyz_yaw msg))
    (cl:cons ':current_xyz_yaw (current_xyz_yaw msg))
    (cl:cons ':desired_velocity (desired_velocity msg))
    (cl:cons ':desired_acceleration (desired_acceleration msg))
    (cl:cons ':cmd (cmd msg))
    (cl:cons ':cmd_limited (cmd_limited msg))
))
