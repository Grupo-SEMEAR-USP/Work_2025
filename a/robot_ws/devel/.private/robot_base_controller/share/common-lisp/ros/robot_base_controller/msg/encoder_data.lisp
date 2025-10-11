; Auto-generated. Do not edit!


(cl:in-package robot_base_controller-msg)


;//! \htmlinclude encoder_data.msg.html

(cl:defclass <encoder_data> (roslisp-msg-protocol:ros-message)
  ((front_right_encoder_data
    :reader front_right_encoder_data
    :initarg :front_right_encoder_data
    :type cl:float
    :initform 0.0)
   (front_left_encoder_data
    :reader front_left_encoder_data
    :initarg :front_left_encoder_data
    :type cl:float
    :initform 0.0)
   (rear_right_encoder_data
    :reader rear_right_encoder_data
    :initarg :rear_right_encoder_data
    :type cl:float
    :initform 0.0)
   (rear_left_encoder_data
    :reader rear_left_encoder_data
    :initarg :rear_left_encoder_data
    :type cl:float
    :initform 0.0))
)

(cl:defclass encoder_data (<encoder_data>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <encoder_data>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'encoder_data)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_base_controller-msg:<encoder_data> is deprecated: use robot_base_controller-msg:encoder_data instead.")))

(cl:ensure-generic-function 'front_right_encoder_data-val :lambda-list '(m))
(cl:defmethod front_right_encoder_data-val ((m <encoder_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_base_controller-msg:front_right_encoder_data-val is deprecated.  Use robot_base_controller-msg:front_right_encoder_data instead.")
  (front_right_encoder_data m))

(cl:ensure-generic-function 'front_left_encoder_data-val :lambda-list '(m))
(cl:defmethod front_left_encoder_data-val ((m <encoder_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_base_controller-msg:front_left_encoder_data-val is deprecated.  Use robot_base_controller-msg:front_left_encoder_data instead.")
  (front_left_encoder_data m))

(cl:ensure-generic-function 'rear_right_encoder_data-val :lambda-list '(m))
(cl:defmethod rear_right_encoder_data-val ((m <encoder_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_base_controller-msg:rear_right_encoder_data-val is deprecated.  Use robot_base_controller-msg:rear_right_encoder_data instead.")
  (rear_right_encoder_data m))

(cl:ensure-generic-function 'rear_left_encoder_data-val :lambda-list '(m))
(cl:defmethod rear_left_encoder_data-val ((m <encoder_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_base_controller-msg:rear_left_encoder_data-val is deprecated.  Use robot_base_controller-msg:rear_left_encoder_data instead.")
  (rear_left_encoder_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <encoder_data>) ostream)
  "Serializes a message object of type '<encoder_data>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'front_right_encoder_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'front_left_encoder_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rear_right_encoder_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rear_left_encoder_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <encoder_data>) istream)
  "Deserializes a message object of type '<encoder_data>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'front_right_encoder_data) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'front_left_encoder_data) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rear_right_encoder_data) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rear_left_encoder_data) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<encoder_data>)))
  "Returns string type for a message object of type '<encoder_data>"
  "robot_base_controller/encoder_data")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'encoder_data)))
  "Returns string type for a message object of type 'encoder_data"
  "robot_base_controller/encoder_data")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<encoder_data>)))
  "Returns md5sum for a message object of type '<encoder_data>"
  "c8bd3a66bc403e0d2c70d6f2f2b5db75")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'encoder_data)))
  "Returns md5sum for a message object of type 'encoder_data"
  "c8bd3a66bc403e0d2c70d6f2f2b5db75")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<encoder_data>)))
  "Returns full string definition for message of type '<encoder_data>"
  (cl:format cl:nil "# encoder_data.msg~%float32 front_right_encoder_data~%float32 front_left_encoder_data~%float32 rear_right_encoder_data~%float32 rear_left_encoder_data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'encoder_data)))
  "Returns full string definition for message of type 'encoder_data"
  (cl:format cl:nil "# encoder_data.msg~%float32 front_right_encoder_data~%float32 front_left_encoder_data~%float32 rear_right_encoder_data~%float32 rear_left_encoder_data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <encoder_data>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <encoder_data>))
  "Converts a ROS message object to a list"
  (cl:list 'encoder_data
    (cl:cons ':front_right_encoder_data (front_right_encoder_data msg))
    (cl:cons ':front_left_encoder_data (front_left_encoder_data msg))
    (cl:cons ':rear_right_encoder_data (rear_right_encoder_data msg))
    (cl:cons ':rear_left_encoder_data (rear_left_encoder_data msg))
))
