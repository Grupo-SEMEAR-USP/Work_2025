;; Auto-generated. Do not edit!


(when (boundp 'robot_base_controller::encoder_data)
  (if (not (find-package "ROBOT_BASE_CONTROLLER"))
    (make-package "ROBOT_BASE_CONTROLLER"))
  (shadow 'encoder_data (find-package "ROBOT_BASE_CONTROLLER")))
(unless (find-package "ROBOT_BASE_CONTROLLER::ENCODER_DATA")
  (make-package "ROBOT_BASE_CONTROLLER::ENCODER_DATA"))

(in-package "ROS")
;;//! \htmlinclude encoder_data.msg.html


(defclass robot_base_controller::encoder_data
  :super ros::object
  :slots (_front_right_encoder_data _front_left_encoder_data _rear_right_encoder_data _rear_left_encoder_data ))

(defmethod robot_base_controller::encoder_data
  (:init
   (&key
    ((:front_right_encoder_data __front_right_encoder_data) 0.0)
    ((:front_left_encoder_data __front_left_encoder_data) 0.0)
    ((:rear_right_encoder_data __rear_right_encoder_data) 0.0)
    ((:rear_left_encoder_data __rear_left_encoder_data) 0.0)
    )
   (send-super :init)
   (setq _front_right_encoder_data (float __front_right_encoder_data))
   (setq _front_left_encoder_data (float __front_left_encoder_data))
   (setq _rear_right_encoder_data (float __rear_right_encoder_data))
   (setq _rear_left_encoder_data (float __rear_left_encoder_data))
   self)
  (:front_right_encoder_data
   (&optional __front_right_encoder_data)
   (if __front_right_encoder_data (setq _front_right_encoder_data __front_right_encoder_data)) _front_right_encoder_data)
  (:front_left_encoder_data
   (&optional __front_left_encoder_data)
   (if __front_left_encoder_data (setq _front_left_encoder_data __front_left_encoder_data)) _front_left_encoder_data)
  (:rear_right_encoder_data
   (&optional __rear_right_encoder_data)
   (if __rear_right_encoder_data (setq _rear_right_encoder_data __rear_right_encoder_data)) _rear_right_encoder_data)
  (:rear_left_encoder_data
   (&optional __rear_left_encoder_data)
   (if __rear_left_encoder_data (setq _rear_left_encoder_data __rear_left_encoder_data)) _rear_left_encoder_data)
  (:serialization-length
   ()
   (+
    ;; float32 _front_right_encoder_data
    4
    ;; float32 _front_left_encoder_data
    4
    ;; float32 _rear_right_encoder_data
    4
    ;; float32 _rear_left_encoder_data
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _front_right_encoder_data
       (sys::poke _front_right_encoder_data (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _front_left_encoder_data
       (sys::poke _front_left_encoder_data (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rear_right_encoder_data
       (sys::poke _rear_right_encoder_data (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rear_left_encoder_data
       (sys::poke _rear_left_encoder_data (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _front_right_encoder_data
     (setq _front_right_encoder_data (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _front_left_encoder_data
     (setq _front_left_encoder_data (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rear_right_encoder_data
     (setq _rear_right_encoder_data (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rear_left_encoder_data
     (setq _rear_left_encoder_data (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get robot_base_controller::encoder_data :md5sum-) "c8bd3a66bc403e0d2c70d6f2f2b5db75")
(setf (get robot_base_controller::encoder_data :datatype-) "robot_base_controller/encoder_data")
(setf (get robot_base_controller::encoder_data :definition-)
      "# encoder_data.msg
float32 front_right_encoder_data
float32 front_left_encoder_data
float32 rear_right_encoder_data
float32 rear_left_encoder_data

")



(provide :robot_base_controller/encoder_data "c8bd3a66bc403e0d2c70d6f2f2b5db75")


