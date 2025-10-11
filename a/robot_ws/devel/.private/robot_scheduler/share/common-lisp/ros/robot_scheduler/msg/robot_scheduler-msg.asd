
(cl:in-package :asdf)

(defsystem "robot_scheduler-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Object" :depends-on ("_package_Object"))
    (:file "_package_Object" :depends-on ("_package"))
    (:file "SchedulerCommand" :depends-on ("_package_SchedulerCommand"))
    (:file "_package_SchedulerCommand" :depends-on ("_package"))
    (:file "SchedulerResponse" :depends-on ("_package_SchedulerResponse"))
    (:file "_package_SchedulerResponse" :depends-on ("_package"))
    (:file "Subtask" :depends-on ("_package_Subtask"))
    (:file "_package_Subtask" :depends-on ("_package"))
    (:file "TaskRequest" :depends-on ("_package_TaskRequest"))
    (:file "_package_TaskRequest" :depends-on ("_package"))
  ))