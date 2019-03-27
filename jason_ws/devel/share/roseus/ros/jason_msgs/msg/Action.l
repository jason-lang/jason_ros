;; Auto-generated. Do not edit!


(when (boundp 'jason_msgs::Action)
  (if (not (find-package "JASON_MSGS"))
    (make-package "JASON_MSGS"))
  (shadow 'Action (find-package "JASON_MSGS")))
(unless (find-package "JASON_MSGS::ACTION")
  (make-package "JASON_MSGS::ACTION"))

(in-package "ROS")
;;//! \htmlinclude Action.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jason_msgs::Action
  :super ros::object
  :slots (_header _action_name _parameters ))

(defmethod jason_msgs::Action
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:action_name __action_name) "")
    ((:parameters __parameters) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _action_name (string __action_name))
   (setq _parameters __parameters)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:action_name
   (&optional __action_name)
   (if __action_name (setq _action_name __action_name)) _action_name)
  (:parameters
   (&optional __parameters)
   (if __parameters (setq _parameters __parameters)) _parameters)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _action_name
    4 (length _action_name)
    ;; string[] _parameters
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _parameters)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _action_name
       (write-long (length _action_name) s) (princ _action_name s)
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
   ;; string _action_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _action_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
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

(setf (get jason_msgs::Action :md5sum-) "60b115de72e5d2f7244b28be9903e13e")
(setf (get jason_msgs::Action :datatype-) "jason_msgs/Action")
(setf (get jason_msgs::Action :definition-)
      "Header header
string action_name
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



(provide :jason_msgs/Action "60b115de72e5d2f7244b28be9903e13e")


