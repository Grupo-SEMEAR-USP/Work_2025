; Auto-generated. Do not edit!


(cl:in-package robot_scheduler-msg)


;//! \htmlinclude TaskRequest.msg.html

(cl:defclass <TaskRequest> (roslisp-msg-protocol:ros-message)
  ((subtasks
    :reader subtasks
    :initarg :subtasks
    :type (cl:vector robot_scheduler-msg:Subtask)
   :initform (cl:make-array 0 :element-type 'robot_scheduler-msg:Subtask :initial-element (cl:make-instance 'robot_scheduler-msg:Subtask)))
   (execute_on
    :reader execute_on
    :initarg :execute_on
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass TaskRequest (<TaskRequest>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TaskRequest>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TaskRequest)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_scheduler-msg:<TaskRequest> is deprecated: use robot_scheduler-msg:TaskRequest instead.")))

(cl:ensure-generic-function 'subtasks-val :lambda-list '(m))
(cl:defmethod subtasks-val ((m <TaskRequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_scheduler-msg:subtasks-val is deprecated.  Use robot_scheduler-msg:subtasks instead.")
  (subtasks m))

(cl:ensure-generic-function 'execute_on-val :lambda-list '(m))
(cl:defmethod execute_on-val ((m <TaskRequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_scheduler-msg:execute_on-val is deprecated.  Use robot_scheduler-msg:execute_on instead.")
  (execute_on m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TaskRequest>) ostream)
  "Serializes a message object of type '<TaskRequest>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'subtasks))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'subtasks))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'execute_on))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'execute_on))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TaskRequest>) istream)
  "Deserializes a message object of type '<TaskRequest>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'subtasks) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'subtasks)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'robot_scheduler-msg:Subtask))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'execute_on) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'execute_on)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TaskRequest>)))
  "Returns string type for a message object of type '<TaskRequest>"
  "robot_scheduler/TaskRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TaskRequest)))
  "Returns string type for a message object of type 'TaskRequest"
  "robot_scheduler/TaskRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TaskRequest>)))
  "Returns md5sum for a message object of type '<TaskRequest>"
  "d4c07a22d46edc9f78c6294bee4a791a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TaskRequest)))
  "Returns md5sum for a message object of type 'TaskRequest"
  "d4c07a22d46edc9f78c6294bee4a791a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TaskRequest>)))
  "Returns full string definition for message of type '<TaskRequest>"
  (cl:format cl:nil "robot_scheduler/Subtask[] subtasks~%string[] execute_on~%~%================================================================================~%MSG: robot_scheduler/Subtask~%robot_scheduler/Object object~%string source~%string destination~%~%================================================================================~%MSG: robot_scheduler/Object~%uint16 object~%uint16 target~%bool decoy~%geometry_msgs/PoseStamped pose~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TaskRequest)))
  "Returns full string definition for message of type 'TaskRequest"
  (cl:format cl:nil "robot_scheduler/Subtask[] subtasks~%string[] execute_on~%~%================================================================================~%MSG: robot_scheduler/Subtask~%robot_scheduler/Object object~%string source~%string destination~%~%================================================================================~%MSG: robot_scheduler/Object~%uint16 object~%uint16 target~%bool decoy~%geometry_msgs/PoseStamped pose~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TaskRequest>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'subtasks) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'execute_on) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TaskRequest>))
  "Converts a ROS message object to a list"
  (cl:list 'TaskRequest
    (cl:cons ':subtasks (subtasks msg))
    (cl:cons ':execute_on (execute_on msg))
))
