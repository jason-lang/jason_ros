;; Auto-generated. Do not edit!


(when (boundp 'jason_msgs::ActionStatus)
  (if (not (find-package "JASON_MSGS"))
    (make-package "JASON_MSGS"))
  (shadow 'ActionStatus (find-package "JASON_MSGS")))
(unless (find-package "JASON_MSGS::ACTIONSTATUS")
  (make-package "JASON_MSGS::ACTIONSTATUS"))

(in-package "ROS")
;;//! \htmlinclude ActionStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jason_msgs::ActionStatus
  :super ros::object
  :slots (_header _result _id ))

(defmethod jason_msgs::ActionStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:result __result) nil)
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _result __result)
   (setq _id (round __id))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _result
    1
    ;; uint32 _id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _result
       (if _result (write-byte -1 s) (write-byte 0 s))
     ;; uint32 _id
       (write-long _id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _result
     (setq _result (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get jason_msgs::ActionStatus :md5sum-) "bd2ff4c1c257386dae20268cf70a828d")
(setf (get jason_msgs::ActionStatus :datatype-) "jason_msgs/ActionStatus")
(setf (get jason_msgs::ActionStatus :definition-)
      "Header header
bool result
uint32 id

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



(provide :jason_msgs/ActionStatus "bd2ff4c1c257386dae20268cf70a828d")


