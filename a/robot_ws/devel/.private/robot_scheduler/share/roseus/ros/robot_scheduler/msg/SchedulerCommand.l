;; Auto-generated. Do not edit!


(when (boundp 'robot_scheduler::SchedulerCommand)
  (if (not (find-package "ROBOT_SCHEDULER"))
    (make-package "ROBOT_SCHEDULER"))
  (shadow 'SchedulerCommand (find-package "ROBOT_SCHEDULER")))
(unless (find-package "ROBOT_SCHEDULER::SCHEDULERCOMMAND")
  (make-package "ROBOT_SCHEDULER::SCHEDULERCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude SchedulerCommand.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass robot_scheduler::SchedulerCommand
  :super ros::object
  :slots (_header _uid _target _payload _need_ack ))

(defmethod robot_scheduler::SchedulerCommand
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:uid __uid) 0)
    ((:target __target) "")
    ((:payload __payload) "")
    ((:need_ack __need_ack) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _uid (round __uid))
   (setq _target (string __target))
   (setq _payload (string __payload))
   (setq _need_ack __need_ack)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:uid
   (&optional __uid)
   (if __uid (setq _uid __uid)) _uid)
  (:target
   (&optional __target)
   (if __target (setq _target __target)) _target)
  (:payload
   (&optional __payload)
   (if __payload (setq _payload __payload)) _payload)
  (:need_ack
   (&optional (__need_ack :null))
   (if (not (eq __need_ack :null)) (setq _need_ack __need_ack)) _need_ack)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint64 _uid
    8
    ;; string _target
    4 (length _target)
    ;; string _payload
    4 (length _payload)
    ;; bool _need_ack
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint64 _uid
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _uid (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _uid) (= (length (_uid . bv)) 2)) ;; bignum
              (write-long (ash (elt (_uid . bv) 0) 0) s)
              (write-long (ash (elt (_uid . bv) 1) -1) s))
             ((and (class _uid) (= (length (_uid . bv)) 1)) ;; big1
              (write-long (elt (_uid . bv) 0) s)
              (write-long (if (>= _uid 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _uid s)(write-long (if (>= _uid 0) 0 #xffffffff) s)))
     ;; string _target
       (write-long (length _target) s) (princ _target s)
     ;; string _payload
       (write-long (length _payload) s) (princ _payload s)
     ;; bool _need_ack
       (if _need_ack (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint64 _uid
     
#+(or :alpha :irix6 :x86_64)
      (setf _uid (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _uid (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; string _target
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _target (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _payload
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _payload (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _need_ack
     (setq _need_ack (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get robot_scheduler::SchedulerCommand :md5sum-) "8c8e7e95e708dac0be8b710a396741c1")
(setf (get robot_scheduler::SchedulerCommand :datatype-) "robot_scheduler/SchedulerCommand")
(setf (get robot_scheduler::SchedulerCommand :definition-)
      "std_msgs/Header header
uint64 uid
string target
string payload
bool need_ack

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

")



(provide :robot_scheduler/SchedulerCommand "8c8e7e95e708dac0be8b710a396741c1")


