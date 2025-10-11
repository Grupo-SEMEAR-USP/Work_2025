;; Auto-generated. Do not edit!


(when (boundp 'robot_scheduler::Subtask)
  (if (not (find-package "ROBOT_SCHEDULER"))
    (make-package "ROBOT_SCHEDULER"))
  (shadow 'Subtask (find-package "ROBOT_SCHEDULER")))
(unless (find-package "ROBOT_SCHEDULER::SUBTASK")
  (make-package "ROBOT_SCHEDULER::SUBTASK"))

(in-package "ROS")
;;//! \htmlinclude Subtask.msg.html


(defclass robot_scheduler::Subtask
  :super ros::object
  :slots (_object _source _destination ))

(defmethod robot_scheduler::Subtask
  (:init
   (&key
    ((:object __object) (instance robot_scheduler::Object :init))
    ((:source __source) "")
    ((:destination __destination) "")
    )
   (send-super :init)
   (setq _object __object)
   (setq _source (string __source))
   (setq _destination (string __destination))
   self)
  (:object
   (&rest __object)
   (if (keywordp (car __object))
       (send* _object __object)
     (progn
       (if __object (setq _object (car __object)))
       _object)))
  (:source
   (&optional __source)
   (if __source (setq _source __source)) _source)
  (:destination
   (&optional __destination)
   (if __destination (setq _destination __destination)) _destination)
  (:serialization-length
   ()
   (+
    ;; robot_scheduler/Object _object
    (send _object :serialization-length)
    ;; string _source
    4 (length _source)
    ;; string _destination
    4 (length _destination)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; robot_scheduler/Object _object
       (send _object :serialize s)
     ;; string _source
       (write-long (length _source) s) (princ _source s)
     ;; string _destination
       (write-long (length _destination) s) (princ _destination s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; robot_scheduler/Object _object
     (send _object :deserialize buf ptr-) (incf ptr- (send _object :serialization-length))
   ;; string _source
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _source (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _destination
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _destination (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get robot_scheduler::Subtask :md5sum-) "6933e02dbcb5080f315b15eafcd4e885")
(setf (get robot_scheduler::Subtask :datatype-) "robot_scheduler/Subtask")
(setf (get robot_scheduler::Subtask :definition-)
      "robot_scheduler/Object object
string source
string destination

================================================================================
MSG: robot_scheduler/Object
uint16 object
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



(provide :robot_scheduler/Subtask "6933e02dbcb5080f315b15eafcd4e885")


