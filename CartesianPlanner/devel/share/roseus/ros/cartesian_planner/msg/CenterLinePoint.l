;; Auto-generated. Do not edit!


(when (boundp 'cartesian_planner::CenterLinePoint)
  (if (not (find-package "CARTESIAN_PLANNER"))
    (make-package "CARTESIAN_PLANNER"))
  (shadow 'CenterLinePoint (find-package "CARTESIAN_PLANNER")))
(unless (find-package "CARTESIAN_PLANNER::CENTERLINEPOINT")
  (make-package "CARTESIAN_PLANNER::CENTERLINEPOINT"))

(in-package "ROS")
;;//! \htmlinclude CenterLinePoint.msg.html


(defclass cartesian_planner::CenterLinePoint
  :super ros::object
  :slots (_s _x _y _theta _kappa _left_bound _right_bound ))

(defmethod cartesian_planner::CenterLinePoint
  (:init
   (&key
    ((:s __s) 0.0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:theta __theta) 0.0)
    ((:kappa __kappa) 0.0)
    ((:left_bound __left_bound) 0.0)
    ((:right_bound __right_bound) 0.0)
    )
   (send-super :init)
   (setq _s (float __s))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _theta (float __theta))
   (setq _kappa (float __kappa))
   (setq _left_bound (float __left_bound))
   (setq _right_bound (float __right_bound))
   self)
  (:s
   (&optional __s)
   (if __s (setq _s __s)) _s)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:kappa
   (&optional __kappa)
   (if __kappa (setq _kappa __kappa)) _kappa)
  (:left_bound
   (&optional __left_bound)
   (if __left_bound (setq _left_bound __left_bound)) _left_bound)
  (:right_bound
   (&optional __right_bound)
   (if __right_bound (setq _right_bound __right_bound)) _right_bound)
  (:serialization-length
   ()
   (+
    ;; float64 _s
    8
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _theta
    8
    ;; float64 _kappa
    8
    ;; float64 _left_bound
    8
    ;; float64 _right_bound
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _s
       (sys::poke _s (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta
       (sys::poke _theta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _kappa
       (sys::poke _kappa (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _left_bound
       (sys::poke _left_bound (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _right_bound
       (sys::poke _right_bound (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _s
     (setq _s (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta
     (setq _theta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _kappa
     (setq _kappa (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _left_bound
     (setq _left_bound (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _right_bound
     (setq _right_bound (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get cartesian_planner::CenterLinePoint :md5sum-) "d8c676686a759a5bfb0f50165bdf3705")
(setf (get cartesian_planner::CenterLinePoint :datatype-) "cartesian_planner/CenterLinePoint")
(setf (get cartesian_planner::CenterLinePoint :definition-)
      "float64 s
float64 x
float64 y
float64 theta
float64 kappa
float64 left_bound
float64 right_bound

")



(provide :cartesian_planner/CenterLinePoint "d8c676686a759a5bfb0f50165bdf3705")


