;; Auto-generated. Do not edit!


(when (boundp 'jason_msgs::Perception)
  (if (not (find-package "JASON_MSGS"))
    (make-package "JASON_MSGS"))
  (shadow 'Perception (find-package "JASON_MSGS")))
(unless (find-package "JASON_MSGS::PERCEPTION")
  (make-package "JASON_MSGS::PERCEPTION"))

(in-package "ROS")
;;//! \htmlinclude Perception.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jason_msgs::Perception
  :super ros::object
  :slots (_header _perception_name _parameters ))

(defmethod jason_msgs::Perception
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:perception_name __perception_name) "")
    ((:parameters __parameters) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _perception_name (string __perception_name))
   (setq _parameters __parameters)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:perception_name
   (&optional __perception_name)
   (if __perception_name (setq _perception_name __perception_name)) _perception_name)
  (:parameters
   (&optional __parameters)
   (if __parameters (setq _parameters __parameters)) _parameters)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _perception_name
    4 (length _perception_name)
    ;; string[] _parameters
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _parameters)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _perception_name
       (write-long (length _perception_name) s) (princ _perception_name s)
     ;; string[] _parameters
     (write-long (length _parameters) s)
     (dolist (elem _parameters)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _perception_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _perception_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string[] _parameters
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _parameters (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _parameters i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(setf (get jason_msgs::Perception :md5sum-) "61b31acc0308141c8fc2599a7f94bcb0")
(setf (get jason_msgs::Perception :datatype-) "jason_msgs/Perception")
(setf (get jason_msgs::Perception :definition-)
      "Header header
string perception_name
string[] parameters

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :jason_msgs/Perception "61b31acc0308141c8fc2599a7f94bcb0")


