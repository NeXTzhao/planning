;; Auto-generated. Do not edit!


(when (boundp 'path::person)
  (if (not (find-package "PATH"))
    (make-package "PATH"))
  (shadow 'person (find-package "PATH")))
(unless (find-package "PATH::PERSON")
  (make-package "PATH::PERSON"))

(in-package "ROS")
;;//! \htmlinclude person.msg.html


(defclass path::person
  :super ros::object
  :slots (_name _sex _age ))

(defmethod path::person
  (:init
   (&key
    ((:name __name) "")
    ((:sex __sex) 0)
    ((:age __age) 0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _sex (round __sex))
   (setq _age (round __age))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:sex
   (&optional __sex)
   (if __sex (setq _sex __sex)) _sex)
  (:age
   (&optional __age)
   (if __age (setq _age __age)) _age)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; uint8 _sex
    1
    ;; uint8 _age
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; uint8 _sex
       (write-byte _sex s)
     ;; uint8 _age
       (write-byte _age s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _sex
     (setq _sex (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _age
     (setq _age (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get path::person :md5sum-) "29c2369226882e65039fc85966db8d02")
(setf (get path::person :datatype-) "path/person")
(setf (get path::person :definition-)
      "
string name
uint8 sex
uint8 age
")



(provide :path/person "29c2369226882e65039fc85966db8d02")


