;; Auto-generated. Do not edit!


(when (boundp 'jason_msgs::ActionStatus)
  (if (not (find-package "JASON_MSGS"))
    (make-package "JASON_MSGS"))
  (shadow 'ActionStatus (find-package "JASON_MSGS")))
(unless (find-package "JASON_MSGS::ACTIONSTATUS")
  (make-package "JASON_MSGS::ACTIONSTATUS"))

(in-package "ROS")
;;//! \htmlinclude ActionStatus.msg.html


(defclass jason_msgs::ActionStatus
  :super ros::object
  :slots ())

(defmethod jason_msgs::ActionStatus
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

(setf (get jason_msgs::ActionStatus :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get jason_msgs::ActionStatus :datatype-) "jason_msgs/ActionStatus")
(setf (get jason_msgs::ActionStatus :definition-)
      "
")



(provide :jason_msgs/ActionStatus "d41d8cd98f00b204e9800998ecf8427e")


