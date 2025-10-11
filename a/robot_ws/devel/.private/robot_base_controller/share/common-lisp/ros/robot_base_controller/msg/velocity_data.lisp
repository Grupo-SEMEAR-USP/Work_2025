; Auto-generated. Do not edit!


(cl:in-package robot_base_controller-msg)


;//! \htmlinclude velocity_data.msg.html

(cl:defclass <velocity_data> (roslisp-msg-protocol:ros-message)
  ((front_left_wheel
    :reader front_left_wheel
    :initarg :front_left_wheel
    :type cl:float
    :initform 0.0)
   (front_right_wheel
    :reader front_right_wheel
    :initarg :front_right_wheel
    :type cl:float
    :initform 0.0)
   (rear_left_wheel
    :reader rear_left_wheel
    :initarg :rear_left_wheel
    :type cl:float
    :initform 0.0)
   (rear_right_wheel
    :reader rear_right_wheel
    :initarg :rear_right_wheel
    :type cl:float
    :initform 0.0))
)

(cl:defclass velocity_data (<velocity_data>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <velocity_data>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'velocity_data)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_base_controller-msg:<velocity_data> is deprecated: use robot_base_controller-msg:velocity_data instead.")))

(cl:ensure-generic-function 'front_left_wheel-val :lambda-list '(m))
(cl:defmethod front_left_wheel-val ((m <velocity_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_base_controller-msg:front_left_wheel-val is deprecated.  Use robot_base_controller-msg:front_left_wheel instead.")
  (front_left_wheel m))

(cl:ensure-generic-function 'front_right_wheel-val :lambda-list '(m))
(cl:defmethod front_right_wheel-val ((m <velocity_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_base_controller-msg:front_right_wheel-val is deprecated.  Use robot_base_controller-msg:front_right_wheel instead.")
  (front_right_wheel m))

(cl:ensure-generic-function 'rear_left_wheel-val :lambda-list '(m))
(cl:defmethod rear_left_wheel-val ((m <velocity_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_base_controller-msg:rear_left_wheel-val is deprecated.  Use robot_base_controller-msg:rear_left_wheel instead.")
  (rear_left_wheel m))

(cl:ensure-generic-function 'rear_right_wheel-val :lambda-list '(m))
(cl:defmethod rear_right_wheel-val ((m <velocity_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_base_controller-msg:rear_right_wheel-val is deprecated.  Use robot_base_controller-msg:rear_right_wheel instead.")
  (rear_right_wheel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <velocity_data>) ostream)
  "Serializes a message object of type '<velocity_data>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'front_left_wheel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'front_right_wheel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rear_left_wheel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rear_right_wheel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <velocity_data>) istream)
  "Deserializes a message object of type '<velocity_data>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'front_left_wheel) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'front_right_wheel) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rear_left_wheel) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rear_right_wheel) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<velocity_data>)))
  "Returns string type for a message object of type '<velocity_data>"
  "robot_base_controller/velocity_data")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'velocity_data)))
  "Returns string type for a message object of type 'velocity_data"
  "robot_base_controller/velocity_data")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<velocity_data>)))
  "Returns md5sum for a message object of type '<velocity_data>"
  "6398bc5e122606acb80413c64aee32c3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'velocity_data)))
  "Returns md5sum for a message object of type 'velocity_data"
  "6398bc5e122606acb80413c64aee32c3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<velocity_data>)))
  "Returns full string definition for message of type '<velocity_data>"
  (cl:format cl:nil "# velocity_data.msg~%float64 front_left_wheel~%float64 front_right_wheel~%float64 rear_left_wheel~%float64 rear_right_wheel~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'velocity_data)))
  "Returns full string definition for message of type 'velocity_data"
  (cl:format cl:nil "# velocity_data.msg~%float64 front_left_wheel~%float64 front_right_wheel~%float64 rear_left_wheel~%float64 rear_right_wheel~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <velocity_data>))
  (cl:+ 0
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <velocity_data>))
  "Converts a ROS message object to a list"
  (cl:list 'velocity_data
    (cl:cons ':front_left_wheel (front_left_wheel msg))
    (cl:cons ':front_right_wheel (front_right_wheel msg))
    (cl:cons ':rear_left_wheel (rear_left_wheel msg))
    (cl:cons ':rear_right_wheel (rear_right_wheel msg))
))
