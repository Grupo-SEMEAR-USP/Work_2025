
(cl:in-package :asdf)

(defsystem "robot_base_controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "encoder_data" :depends-on ("_package_encoder_data"))
    (:file "_package_encoder_data" :depends-on ("_package"))
    (:file "velocity_data" :depends-on ("_package_velocity_data"))
    (:file "_package_velocity_data" :depends-on ("_package"))
  ))