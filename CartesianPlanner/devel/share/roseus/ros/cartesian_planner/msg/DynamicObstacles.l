;; Auto-generated. Do not edit!


(when (boundp 'cartesian_planner::DynamicObstacles)
  (if (not (find-package "CARTESIAN_PLANNER"))
    (make-package "CARTESIAN_PLANNER"))
  (shadow 'DynamicObstacles (find-package "CARTESIAN_PLANNER")))
(unless (find-package "CARTESIAN_PLANNER::DYNAMICOBSTACLES")
  (make-package "CARTESIAN_PLANNER::DYNAMICOBSTACLES"))

(in-package "ROS")
;;//! \htmlinclude DynamicObstacles.msg.html


(defclass cartesian_planner::DynamicObstacles
  :super ros::object
  :slots (_obstacles ))

(defmethod cartesian_planner::DynamicObstacles
  (:init
   (&key
    ((:obstacles __obstacles) ())
    )
   (send-super :init)
   (setq _obstacles __obstacles)
   self)
  (:obstacles
   (&rest __obstacles)
   (if (keywordp (car __obstacles))
       (send* _obstacles __obstacles)
     (progn
       (if __obstacles (setq _obstacles (car __obstacles)))
       _obstacles)))
  (:serialization-length
   ()
   (+
    ;; cartesian_planner/DynamicObstacle[] _obstacles
    (apply #'+ (send-all _obstacles :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; cartesian_planner/DynamicObstacle[] _obstacles
     (write-long (length _obstacles) s)
     (dolist (elem _obstacles)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; cartesian_planner/DynamicObstacle[] _obstacles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obstacles (let (r) (dotimes (i n) (push (instance cartesian_planner::DynamicObstacle :init) r)) r))
     (dolist (elem- _obstacles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get cartesian_planner::DynamicObstacles :md5sum-) "30b207dd63b971b7a3a28ba13f187795")
(setf (get cartesian_planner::DynamicObstacles :datatype-) "cartesian_planner/DynamicObstacles")
(setf (get cartesian_planner::DynamicObstacles :definition-)
      "DynamicObstacle[] obstacles

================================================================================
MSG: cartesian_planner/DynamicObstacle
geometry_msgs/Polygon polygon
DynamicTrajectoryPoint[] trajectory

================================================================================
MSG: geometry_msgs/Polygon
#A specification of a polygon where the first and last points are assumed to be connected
Point32[] points

================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
float32 z
================================================================================
MSG: cartesian_planner/DynamicTrajectoryPoint
float64 time
float64 x
float64 y
float64 theta

")



(provide :cartesian_planner/DynamicObstacles "30b207dd63b971b7a3a28ba13f187795")


