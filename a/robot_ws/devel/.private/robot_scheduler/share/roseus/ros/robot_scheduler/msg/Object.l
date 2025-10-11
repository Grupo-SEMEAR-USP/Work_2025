;; Auto-generated. Do not edit!


(when (boundp 'robot_scheduler::Object)
  (if (not (find-package "ROBOT_SCHEDULER"))
    (make-package "ROBOT_SCHEDULER"))
  (shadow 'Object (find-package "ROBOT_SCHEDULER")))
(unless (find-package "ROBOT_SCHEDULER::OBJECT")
  (make-package "ROBOT_SCHEDULER::OBJECT"))

(in-package "ROS")
;;//! \htmlinclude Object.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass robot_scheduler::Object
  :super ros::object
  :slots (_object _target _decoy _pose ))

(defmethod robot_scheduler::Object
  (:init
   (&key
    ((:object __object) 0)
    ((:target __target) 0)
    ((:decoy __decoy) nil)
    ((:pose __pose) (instance geometry_msgs::PoseStamped :init))
    )
   (send-super :init)
   (setq _object (round __object))
   (setq _target (round __target))
   (setq _decoy __decoy)
   (setq _pose __pose)
   self)
  (:object
   (&optional __object)
   (if __object (setq _object __object)) _object)
  (:target
   (&optional __target)
   (if __target (setq _target __target)) _target)
  (:decoy
   (&optional (__decoy :null))
   (if (not (eq __decoy :null)) (setq _decoy __decoy)) _decoy)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:serialization-length
   ()
   (+
    ;; uint16 _object
    2
    ;; uint16 _target
    2
    ;; bool _decoy
    1
    ;; geometry_msgs/PoseStamped _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _object
       (write-word _object s)
     ;; uint16 _target
       (write-word _target s)
     ;; bool _decoy
       (if _decoy (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/PoseStamped _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _object
     (setq _object (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _target
     (setq _target (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; bool _decoy
     (setq _decoy (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/PoseStamped _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(setf (get robot_scheduler::Object :md5sum-) "18be81d5b846c94202e651e2dbb94986")
(setf (get robot_scheduler::Object :datatype-) "robot_scheduler/Object")
(setf (get robot_scheduler::Object :definition-)
      "uint16 object
uint16 target
bool decoy
geometry_msgs/PoseStamped pose

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :robot_scheduler/Object "18be81d5b846c94202e651e2dbb94986")


