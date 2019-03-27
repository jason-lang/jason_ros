;; Auto-generated. Do not edit!


(when (boundp 'jason_msgs::Perception)
  (if (not (find-package "JASON_MSGS"))
    (make-package "JASON_MSGS"))
  (shadow 'Perception (find-package "JASON_MSGS")))
(unless (find-package "JASON_MSGS::PERCEPTION")
  (make-package "JASON_MSGS::PERCEPTION"))

(in-package "ROS")
;;//! \htmlinclude Perception.msg.html


(defclass jason_msgs::Perception
  :super ros::object
  :slots ())

(defmethod jason_msgs::Perception
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get jason_msgs::Perception :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get jason_msgs::Perception :datatype-) "jason_msgs/Perception")
(setf (get jason_msgs::Perception :definition-)
      "
")



(provide :jason_msgs/Perception "d41d8cd98f00b204e9800998ecf8427e")


