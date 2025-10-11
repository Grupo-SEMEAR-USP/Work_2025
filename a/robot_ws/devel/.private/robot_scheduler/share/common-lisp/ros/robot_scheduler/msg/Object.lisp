; Auto-generated. Do not edit!


(cl:in-package robot_scheduler-msg)


;//! \htmlinclude Object.msg.html

(cl:defclass <Object> (roslisp-msg-protocol:ros-message)
  ((object
    :reader object
    :initarg :object
    :type cl:fixnum
    :initform 0)
   (target
    :reader target
    :initarg :target
    :type cl:fixnum
    :initform 0)
   (decoy
    :reader decoy
    :initarg :decoy
    :type cl:boolean
    :initform cl:nil)
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
)

(cl:defclass Object (<Object>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Object>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Object)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_scheduler-msg:<Object> is deprecated: use robot_scheduler-msg:Object instead.")))

(cl:ensure-generic-function 'object-val :lambda-list '(m))
(cl:defmethod object-val ((m <Object>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_scheduler-msg:object-val is deprecated.  Use robot_scheduler-msg:object instead.")
  (object m))

(cl:ensure-generic-function 'target-val :lambda-list '(m))
(cl:defmethod target-val ((m <Object>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_scheduler-msg:target-val is deprecated.  Use robot_scheduler-msg:target instead.")
  (target m))

(cl:ensure-generic-function 'decoy-val :lambda-list '(m))
(cl:defmethod decoy-val ((m <Object>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_scheduler-msg:decoy-val is deprecated.  Use robot_scheduler-msg:decoy instead.")
  (decoy m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <Object>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_scheduler-msg:pose-val is deprecated.  Use robot_scheduler-msg:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Object>) ostream)
  "Serializes a message object of type '<Object>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'object)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'object)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'target)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'target)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'decoy) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Object>) istream)
  "Deserializes a message object of type '<Object>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'object)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'object)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'target)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'target)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'decoy) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Object>)))
  "Returns string type for a message object of type '<Object>"
  "robot_scheduler/Object")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Object)))
  "Returns string type for a message object of type 'Object"
  "robot_scheduler/Object")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Object>)))
  "Returns md5sum for a message object of type '<Object>"
  "18be81d5b846c94202e651e2dbb94986")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Object)))
  "Returns md5sum for a message object of type 'Object"
  "18be81d5b846c94202e651e2dbb94986")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Object>)))
  "Returns full string definition for message of type '<Object>"
  (cl:format cl:nil "uint16 object~%uint16 target~%bool decoy~%geometry_msgs/PoseStamped pose~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Object)))
  "Returns full string definition for message of type 'Object"
  (cl:format cl:nil "uint16 object~%uint16 target~%bool decoy~%geometry_msgs/PoseStamped pose~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Object>))
  (cl:+ 0
     2
     2
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Object>))
  "Converts a ROS message object to a list"
  (cl:list 'Object
    (cl:cons ':object (object msg))
    (cl:cons ':target (target msg))
    (cl:cons ':decoy (decoy msg))
    (cl:cons ':pose (pose msg))
))
