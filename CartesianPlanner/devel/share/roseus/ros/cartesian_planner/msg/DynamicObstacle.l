;; Auto-generated. Do not edit!


(when (boundp 'cartesian_planner::DynamicObstacle)
  (if (not (find-package "CARTESIAN_PLANNER"))
    (make-package "CARTESIAN_PLANNER"))
  (shadow 'DynamicObstacle (find-package "CARTESIAN_PLANNER")))
(unless (find-package "CARTESIAN_PLANNER::DYNAMICOBSTACLE")
  (make-package "CARTESIAN_PLANNER::DYNAMICOBSTACLE"))

(in-package "ROS")
;;//! \htmlinclude DynamicObstacle.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass cartesian_planner::DynamicObstacle
  :super ros::object
  :slots (_polygon _trajectory ))

(defmethod cartesian_planner::DynamicObstacle
  (:init
   (&key
    ((:polygon __polygon) (instance geometry_msgs::Polygon :init))
    ((:trajectory __trajectory) ())
    )
   (send-super :init)
   (setq _polygon __polygon)
   (setq _trajectory __trajectory)
   self)
  (:polygon
   (&rest __polygon)
   (if (keywordp (car __polygon))
       (send* _polygon __polygon)
     (progn
       (if __polygon (setq _polygon (car __polygon)))
       _polygon)))
  (:trajectory
   (&rest __trajectory)
   (if (keywordp (car __trajectory))
       (send* _trajectory __trajectory)
     (progn
       (if __trajectory (setq _trajectory (car __trajectory)))
       _trajectory)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Polygon _polygon
    (send _polygon :serialization-length)
    ;; cartesian_planner/DynamicTrajectoryPoint[] _trajectory
    (apply #'+ (send-all _trajectory :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Polygon _polygon
       (send _polygon :serialize s)
     ;; cartesian_planner/DynamicTrajectoryPoint[] _trajectory
     (write-long (length _trajectory) s)
     (dolist (elem _trajectory)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Polygon _polygon
     (send _polygon :deserialize buf ptr-) (incf ptr- (send _polygon :serialization-length))
   ;; cartesian_planner/DynamicTrajectoryPoint[] _trajectory
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _trajectory (let (r) (dotimes (i n) (push (instance cartesian_planner::DynamicTrajectoryPoint :init) r)) r))
     (dolist (elem- _trajectory)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get cartesian_planner::DynamicObstacle :md5sum-) "9cbe4735150eb164552f0f587e68e631")
(setf (get cartesian_planner::DynamicObstacle :datatype-) "cartesian_planner/DynamicObstacle")
(setf (get cartesian_planner::DynamicObstacle :definition-)
      "geometry_msgs/Polygon polygon
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



(provide :cartesian_planner/DynamicObstacle "9cbe4735150eb164552f0f587e68e631")


