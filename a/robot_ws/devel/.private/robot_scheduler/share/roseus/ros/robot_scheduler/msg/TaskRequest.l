;; Auto-generated. Do not edit!


(when (boundp 'robot_scheduler::TaskRequest)
  (if (not (find-package "ROBOT_SCHEDULER"))
    (make-package "ROBOT_SCHEDULER"))
  (shadow 'TaskRequest (find-package "ROBOT_SCHEDULER")))
(unless (find-package "ROBOT_SCHEDULER::TASKREQUEST")
  (make-package "ROBOT_SCHEDULER::TASKREQUEST"))

(in-package "ROS")
;;//! \htmlinclude TaskRequest.msg.html


(defclass robot_scheduler::TaskRequest
  :super ros::object
  :slots (_subtasks _execute_on ))

(defmethod robot_scheduler::TaskRequest
  (:init
   (&key
    ((:subtasks __subtasks) ())
    ((:execute_on __execute_on) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _subtasks __subtasks)
   (setq _execute_on __execute_on)
   self)
  (:subtasks
   (&rest __subtasks)
   (if (keywordp (car __subtasks))
       (send* _subtasks __subtasks)
     (progn
       (if __subtasks (setq _subtasks (car __subtasks)))
       _subtasks)))
  (:execute_on
   (&optional __execute_on)
   (if __execute_on (setq _execute_on __execute_on)) _execute_on)
  (:serialization-length
   ()
   (+
    ;; robot_scheduler/Subtask[] _subtasks
    (apply #'+ (send-all _subtasks :serialization-length)) 4
    ;; string[] _execute_on
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _execute_on)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; robot_scheduler/Subtask[] _subtasks
     (write-long (length _subtasks) s)
     (dolist (elem _subtasks)
       (send elem :serialize s)
       )
     ;; string[] _execute_on
     (write-long (length _execute_on) s)
     (dolist (elem _execute_on)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; robot_scheduler/Subtask[] _subtasks
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _subtasks (let (r) (dotimes (i n) (push (instance robot_scheduler::Subtask :init) r)) r))
     (dolist (elem- _subtasks)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; string[] _execute_on
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _execute_on (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _execute_on i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(setf (get robot_scheduler::TaskRequest :md5sum-) "d4c07a22d46edc9f78c6294bee4a791a")
(setf (get robot_scheduler::TaskRequest :datatype-) "robot_scheduler/TaskRequest")
(setf (get robot_scheduler::TaskRequest :definition-)
      "robot_scheduler/Subtask[] subtasks
string[] execute_on

================================================================================
MSG: robot_scheduler/Subtask
robot_scheduler/Object object
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



(provide :robot_scheduler/TaskRequest "d4c07a22d46edc9f78c6294bee4a791a")


