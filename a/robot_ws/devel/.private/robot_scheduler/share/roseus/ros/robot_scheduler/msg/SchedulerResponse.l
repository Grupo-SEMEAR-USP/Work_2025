;; Auto-generated. Do not edit!


(when (boundp 'robot_scheduler::SchedulerResponse)
  (if (not (find-package "ROBOT_SCHEDULER"))
    (make-package "ROBOT_SCHEDULER"))
  (shadow 'SchedulerResponse (find-package "ROBOT_SCHEDULER")))
(unless (find-package "ROBOT_SCHEDULER::SCHEDULERRESPONSE")
  (make-package "ROBOT_SCHEDULER::SCHEDULERRESPONSE"))

(in-package "ROS")
;;//! \htmlinclude SchedulerResponse.msg.html


(defclass robot_scheduler::SchedulerResponse
  :super ros::object
  :slots (_uid _target _status ))

(defmethod robot_scheduler::SchedulerResponse
  (:init
   (&key
    ((:uid __uid) 0)
    ((:target __target) "")
    ((:status __status) "")
    )
   (send-super :init)
   (setq _uid (round __uid))
   (setq _target (string __target))
   (setq _status (string __status))
   self)
  (:uid
   (&optional __uid)
   (if __uid (setq _uid __uid)) _uid)
  (:target
   (&optional __target)
   (if __target (setq _target __target)) _target)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; uint64 _uid
    8
    ;; string _target
    4 (length _target)
    ;; string _status
    4 (length _status)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
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
     ;; string _status
       (write-long (length _status) s) (princ _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
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
   ;; string _status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get robot_scheduler::SchedulerResponse :md5sum-) "e762c6baa6b0e70c2b053bc0f5604848")
(setf (get robot_scheduler::SchedulerResponse :datatype-) "robot_scheduler/SchedulerResponse")
(setf (get robot_scheduler::SchedulerResponse :definition-)
      "uint64 uid
string target
string status      

")



(provide :robot_scheduler/SchedulerResponse "e762c6baa6b0e70c2b053bc0f5604848")


