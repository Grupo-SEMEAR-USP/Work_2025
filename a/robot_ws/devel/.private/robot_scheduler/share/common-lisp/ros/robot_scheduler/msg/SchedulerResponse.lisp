; Auto-generated. Do not edit!


(cl:in-package robot_scheduler-msg)


;//! \htmlinclude SchedulerResponse.msg.html

(cl:defclass <SchedulerResponse> (roslisp-msg-protocol:ros-message)
  ((uid
    :reader uid
    :initarg :uid
    :type cl:integer
    :initform 0)
   (target
    :reader target
    :initarg :target
    :type cl:string
    :initform "")
   (status
    :reader status
    :initarg :status
    :type cl:string
    :initform ""))
)

(cl:defclass SchedulerResponse (<SchedulerResponse>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SchedulerResponse>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SchedulerResponse)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_scheduler-msg:<SchedulerResponse> is deprecated: use robot_scheduler-msg:SchedulerResponse instead.")))

(cl:ensure-generic-function 'uid-val :lambda-list '(m))
(cl:defmethod uid-val ((m <SchedulerResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_scheduler-msg:uid-val is deprecated.  Use robot_scheduler-msg:uid instead.")
  (uid m))

(cl:ensure-generic-function 'target-val :lambda-list '(m))
(cl:defmethod target-val ((m <SchedulerResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_scheduler-msg:target-val is deprecated.  Use robot_scheduler-msg:target instead.")
  (target m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <SchedulerResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_scheduler-msg:status-val is deprecated.  Use robot_scheduler-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SchedulerResponse>) ostream)
  "Serializes a message object of type '<SchedulerResponse>"
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
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SchedulerResponse>) istream)
  "Deserializes a message object of type '<SchedulerResponse>"
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
      (cl:setf (cl:slot-value msg 'status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SchedulerResponse>)))
  "Returns string type for a message object of type '<SchedulerResponse>"
  "robot_scheduler/SchedulerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SchedulerResponse)))
  "Returns string type for a message object of type 'SchedulerResponse"
  "robot_scheduler/SchedulerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SchedulerResponse>)))
  "Returns md5sum for a message object of type '<SchedulerResponse>"
  "e762c6baa6b0e70c2b053bc0f5604848")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SchedulerResponse)))
  "Returns md5sum for a message object of type 'SchedulerResponse"
  "e762c6baa6b0e70c2b053bc0f5604848")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SchedulerResponse>)))
  "Returns full string definition for message of type '<SchedulerResponse>"
  (cl:format cl:nil "uint64 uid~%string target~%string status      ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SchedulerResponse)))
  "Returns full string definition for message of type 'SchedulerResponse"
  (cl:format cl:nil "uint64 uid~%string target~%string status      ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SchedulerResponse>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'target))
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SchedulerResponse>))
  "Converts a ROS message object to a list"
  (cl:list 'SchedulerResponse
    (cl:cons ':uid (uid msg))
    (cl:cons ':target (target msg))
    (cl:cons ':status (status msg))
))
