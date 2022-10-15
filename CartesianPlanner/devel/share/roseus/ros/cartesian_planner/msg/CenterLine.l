;; Auto-generated. Do not edit!


(when (boundp 'cartesian_planner::CenterLine)
  (if (not (find-package "CARTESIAN_PLANNER"))
    (make-package "CARTESIAN_PLANNER"))
  (shadow 'CenterLine (find-package "CARTESIAN_PLANNER")))
(unless (find-package "CARTESIAN_PLANNER::CENTERLINE")
  (make-package "CARTESIAN_PLANNER::CENTERLINE"))

(in-package "ROS")
;;//! \htmlinclude CenterLine.msg.html


(defclass cartesian_planner::CenterLine
  :super ros::object
  :slots (_points ))

(defmethod cartesian_planner::CenterLine
  (:init
   (&key
    ((:points __points) ())
    )
   (send-super :init)
   (setq _points __points)
   self)
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:serialization-length
   ()
   (+
    ;; cartesian_planner/CenterLinePoint[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; cartesian_planner/CenterLinePoint[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; cartesian_planner/CenterLinePoint[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance cartesian_planner::CenterLinePoint :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get cartesian_planner::CenterLine :md5sum-) "76f98b257c0b7d1218f95661429b5529")
(setf (get cartesian_planner::CenterLine :datatype-) "cartesian_planner/CenterLine")
(setf (get cartesian_planner::CenterLine :definition-)
      "CenterLinePoint[] points

================================================================================
MSG: cartesian_planner/CenterLinePoint
float64 s
float64 x
float64 y
float64 theta
float64 kappa
float64 left_bound
float64 right_bound

")



(provide :cartesian_planner/CenterLine "76f98b257c0b7d1218f95661429b5529")


