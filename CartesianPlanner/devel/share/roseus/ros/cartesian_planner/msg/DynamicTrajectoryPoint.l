;; Auto-generated. Do not edit!


(when (boundp 'cartesian_planner::DynamicTrajectoryPoint)
  (if (not (find-package "CARTESIAN_PLANNER"))
    (make-package "CARTESIAN_PLANNER"))
  (shadow 'DynamicTrajectoryPoint (find-package "CARTESIAN_PLANNER")))
(unless (find-package "CARTESIAN_PLANNER::DYNAMICTRAJECTORYPOINT")
  (make-package "CARTESIAN_PLANNER::DYNAMICTRAJECTORYPOINT"))

(in-package "ROS")
;;//! \htmlinclude DynamicTrajectoryPoint.msg.html


(defclass cartesian_planner::DynamicTrajectoryPoint
  :super ros::object
  :slots (_time _x _y _theta ))

(defmethod cartesian_planner::DynamicTrajectoryPoint
  (:init
   (&key
    ((:time __time) 0.0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:theta __theta) 0.0)
    )
   (send-super :init)
   (setq _time (float __time))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _theta (float __theta))
   self)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:serialization-length
   ()
   (+
    ;; float64 _time
    8
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _theta
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _time
       (sys::poke _time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta
       (sys::poke _theta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _time
     (setq _time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta
     (setq _theta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get cartesian_planner::DynamicTrajectoryPoint :md5sum-) "3500997421bb06e40b442d83f83150dc")
(setf (get cartesian_planner::DynamicTrajectoryPoint :datatype-) "cartesian_planner/DynamicTrajectoryPoint")
(setf (get cartesian_planner::DynamicTrajectoryPoint :definition-)
      "float64 time
float64 x
float64 y
float64 theta

")



(provide :cartesian_planner/DynamicTrajectoryPoint "3500997421bb06e40b442d83f83150dc")


