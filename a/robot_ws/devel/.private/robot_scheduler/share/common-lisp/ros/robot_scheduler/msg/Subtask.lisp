; Auto-generated. Do not edit!


(cl:in-package robot_scheduler-msg)


;//! \htmlinclude Subtask.msg.html

(cl:defclass <Subtask> (roslisp-msg-protocol:ros-message)
  ((object
    :reader object
    :initarg :object
    :type robot_scheduler-msg:Object
    :initform (cl:make-instance 'robot_scheduler-msg:Object))
   (source
    :reader source
    :initarg :source
    :type cl:string
    :initform "")
   (destination
    :reader destination
    :initarg :destination
    :type cl:string
    :initform ""))
)

(cl:defclass Subtask (<Subtask>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Subtask>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Subtask)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_scheduler-msg:<Subtask> is deprecated: use robot_scheduler-msg:Subtask instead.")))

(cl:ensure-generic-function 'object-val :lambda-list '(m))
(cl:defmethod object-val ((m <Subtask>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_scheduler-msg:object-val is deprecated.  Use robot_scheduler-msg:object instead.")
  (object m))

(cl:ensure-generic-function 'source-val :lambda-list '(m))
(cl:defmethod source-val ((m <Subtask>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_scheduler-msg:source-val is deprecated.  Use robot_scheduler-msg:source instead.")
  (source m))

(cl:ensure-generic-function 'destination-val :lambda-list '(m))
(cl:defmethod destination-val ((m <Subtask>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_scheduler-msg:destination-val is deprecated.  Use robot_scheduler-msg:destination instead.")
  (destination m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Subtask>) ostream)
  "Serializes a message object of type '<Subtask>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'object) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'source))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'source))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'destination))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'destination))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Subtask>) istream)
  "Deserializes a message object of type '<Subtask>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'object) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'source) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'source) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'destination) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'destination) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Subtask>)))
  "Returns string type for a message object of type '<Subtask>"
  "robot_scheduler/Subtask")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Subtask)))
  "Returns string type for a message object of type 'Subtask"
  "robot_scheduler/Subtask")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Subtask>)))
  "Returns md5sum for a message object of type '<Subtask>"
  "6933e02dbcb5080f315b15eafcd4e885")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Subtask)))
  "Returns md5sum for a message object of type 'Subtask"
  "6933e02dbcb5080f315b15eafcd4e885")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Subtask>)))
  "Returns full string definition for message of type '<Subtask>"
  (cl:format cl:nil "robot_scheduler/Object object~%string source~%string destination~%~%================================================================================~%MSG: robot_scheduler/Object~%uint16 object~%uint16 target~%bool decoy~%geometry_msgs/PoseStamped pose~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Subtask)))
  "Returns full string definition for message of type 'Subtask"
  (cl:format cl:nil "robot_scheduler/Object object~%string source~%string destination~%~%================================================================================~%MSG: robot_scheduler/Object~%uint16 object~%uint16 target~%bool decoy~%geometry_msgs/PoseStamped pose~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Subtask>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'object))
     4 (cl:length (cl:slot-value msg 'source))
     4 (cl:length (cl:slot-value msg 'destination))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Subtask>))
  "Converts a ROS message object to a list"
  (cl:list 'Subtask
    (cl:cons ':object (object msg))
    (cl:cons ':source (source msg))
    (cl:cons ':destination (destination msg))
))
