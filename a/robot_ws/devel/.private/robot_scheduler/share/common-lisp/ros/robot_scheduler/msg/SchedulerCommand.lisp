; Auto-generated. Do not edit!


(cl:in-package robot_scheduler-msg)


;//! \htmlinclude SchedulerCommand.msg.html

(cl:defclass <SchedulerCommand> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (uid
    :reader uid
    :initarg :uid
    :type cl:integer
    :initform 0)
   (target
    :reader target
    :initarg :target
    :type cl:string
    :initform "")
   (payload
    :reader payload
    :initarg :payload
    :type cl:string
    :initform "")
   (need_ack
    :reader need_ack
    :initarg :need_ack
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SchedulerCommand (<SchedulerCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SchedulerCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SchedulerCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_scheduler-msg:<SchedulerCommand> is deprecated: use robot_scheduler-msg:SchedulerCommand instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SchedulerCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_scheduler-msg:header-val is deprecated.  Use robot_scheduler-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'uid-val :lambda-list '(m))
(cl:defmethod uid-val ((m <SchedulerCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_scheduler-msg:uid-val is deprecated.  Use robot_scheduler-msg:uid instead.")
  (uid m))

(cl:ensure-generic-function 'target-val :lambda-list '(m))
(cl:defmethod target-val ((m <SchedulerCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_scheduler-msg:target-val is deprecated.  Use robot_scheduler-msg:target instead.")
  (target m))

(cl:ensure-generic-function 'payload-val :lambda-list '(m))
(cl:defmethod payload-val ((m <SchedulerCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_scheduler-msg:payload-val is deprecated.  Use robot_scheduler-msg:payload instead.")
  (payload m))

(cl:ensure-generic-function 'need_ack-val :lambda-list '(m))
(cl:defmethod need_ack-val ((m <SchedulerCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_scheduler-msg:need_ack-val is deprecated.  Use robot_scheduler-msg:need_ack instead.")
  (need_ack m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SchedulerCommand>) ostream)
  "Serializes a message object of type '<SchedulerCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'uid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'uid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'uid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'uid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'uid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'uid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'uid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'uid)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'target))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'target))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'payload))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'payload))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'need_ack) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SchedulerCommand>) istream)
  "Deserializes a message object of type '<SchedulerCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'uid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'uid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'uid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'uid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'uid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'uid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'uid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'uid)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'target) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'target) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'payload) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'payload) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'need_ack) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SchedulerCommand>)))
  "Returns string type for a message object of type '<SchedulerCommand>"
  "robot_scheduler/SchedulerCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SchedulerCommand)))
  "Returns string type for a message object of type 'SchedulerCommand"
  "robot_scheduler/SchedulerCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SchedulerCommand>)))
  "Returns md5sum for a message object of type '<SchedulerCommand>"
  "8c8e7e95e708dac0be8b710a396741c1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SchedulerCommand)))
  "Returns md5sum for a message object of type 'SchedulerCommand"
  "8c8e7e95e708dac0be8b710a396741c1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SchedulerCommand>)))
  "Returns full string definition for message of type '<SchedulerCommand>"
  (cl:format cl:nil "std_msgs/Header header~%uint64 uid~%string target~%string payload~%bool need_ack~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SchedulerCommand)))
  "Returns full string definition for message of type 'SchedulerCommand"
  (cl:format cl:nil "std_msgs/Header header~%uint64 uid~%string target~%string payload~%bool need_ack~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SchedulerCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     4 (cl:length (cl:slot-value msg 'target))
     4 (cl:length (cl:slot-value msg 'payload))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SchedulerCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'SchedulerCommand
    (cl:cons ':header (header msg))
    (cl:cons ':uid (uid msg))
    (cl:cons ':target (target msg))
    (cl:cons ':payload (payload msg))
    (cl:cons ':need_ack (need_ack msg))
))
