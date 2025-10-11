;; Auto-generated. Do not edit!


(when (boundp 'robot_base_controller::velocity_data)
  (if (not (find-package "ROBOT_BASE_CONTROLLER"))
    (make-package "ROBOT_BASE_CONTROLLER"))
  (shadow 'velocity_data (find-package "ROBOT_BASE_CONTROLLER")))
(unless (find-package "ROBOT_BASE_CONTROLLER::VELOCITY_DATA")
  (make-package "ROBOT_BASE_CONTROLLER::VELOCITY_DATA"))

(in-package "ROS")
;;//! \htmlinclude velocity_data.msg.html


(defclass robot_base_controller::velocity_data
  :super ros::object
  :slots (_front_left_wheel _front_right_wheel _rear_left_wheel _rear_right_wheel ))

(defmethod robot_base_controller::velocity_data
  (:init
   (&key
    ((:front_left_wheel __front_left_wheel) 0.0)
    ((:front_right_wheel __front_right_wheel) 0.0)
    ((:rear_left_wheel __rear_left_wheel) 0.0)
    ((:rear_right_wheel __rear_right_wheel) 0.0)
    )
   (send-super :init)
   (setq _front_left_wheel (float __front_left_wheel))
   (setq _front_right_wheel (float __front_right_wheel))
   (setq _rear_left_wheel (float __rear_left_wheel))
   (setq _rear_right_wheel (float __rear_right_wheel))
   self)
  (:front_left_wheel
   (&optional __front_left_wheel)
   (if __front_left_wheel (setq _front_left_wheel __front_left_wheel)) _front_left_wheel)
  (:front_right_wheel
   (&optional __front_right_wheel)
   (if __front_right_wheel (setq _front_right_wheel __front_right_wheel)) _front_right_wheel)
  (:rear_left_wheel
   (&optional __rear_left_wheel)
   (if __rear_left_wheel (setq _rear_left_wheel __rear_left_wheel)) _rear_left_wheel)
  (:rear_right_wheel
   (&optional __rear_right_wheel)
   (if __rear_right_wheel (setq _rear_right_wheel __rear_right_wheel)) _rear_right_wheel)
  (:serialization-length
   ()
   (+
    ;; float64 _front_left_wheel
    8
    ;; float64 _front_right_wheel
    8
    ;; float64 _rear_left_wheel
    8
    ;; float64 _rear_right_wheel
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _front_left_wheel
       (sys::poke _front_left_wheel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _front_right_wheel
       (sys::poke _front_right_wheel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rear_left_wheel
       (sys::poke _rear_left_wheel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rear_right_wheel
       (sys::poke _rear_right_wheel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _front_left_wheel
     (setq _front_left_wheel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _front_right_wheel
     (setq _front_right_wheel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rear_left_wheel
     (setq _rear_left_wheel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rear_right_wheel
     (setq _rear_right_wheel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get robot_base_controller::velocity_data :md5sum-) "6398bc5e122606acb80413c64aee32c3")
(setf (get robot_base_controller::velocity_data :datatype-) "robot_base_controller/velocity_data")
(setf (get robot_base_controller::velocity_data :definition-)
      "# velocity_data.msg
float64 front_left_wheel
float64 front_right_wheel
float64 rear_left_wheel
float64 rear_right_wheel
")



(provide :robot_base_controller/velocity_data "6398bc5e122606acb80413c64aee32c3")


