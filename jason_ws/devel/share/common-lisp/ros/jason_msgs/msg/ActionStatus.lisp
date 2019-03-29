; Auto-generated. Do not edit!


(cl:in-package jason_msgs-msg)


;//! \htmlinclude ActionStatus.msg.html

(cl:defclass <ActionStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil)
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0))
)

(cl:defclass ActionStatus (<ActionStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActionStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActionStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jason_msgs-msg:<ActionStatus> is deprecated: use jason_msgs-msg:ActionStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ActionStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jason_msgs-msg:header-val is deprecated.  Use jason_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <ActionStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jason_msgs-msg:result-val is deprecated.  Use jason_msgs-msg:result instead.")
  (result m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <ActionStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jason_msgs-msg:id-val is deprecated.  Use jason_msgs-msg:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActionStatus>) ostream)
  "Serializes a message object of type '<ActionStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActionStatus>) istream)
  "Deserializes a message object of type '<ActionStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ActionStatus>)))
  "Returns string type for a message object of type '<ActionStatus>"
  "jason_msgs/ActionStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActionStatus)))
  "Returns string type for a message object of type 'ActionStatus"
  "jason_msgs/ActionStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ActionStatus>)))
  "Returns md5sum for a message object of type '<ActionStatus>"
  "bd2ff4c1c257386dae20268cf70a828d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActionStatus)))
  "Returns md5sum for a message object of type 'ActionStatus"
  "bd2ff4c1c257386dae20268cf70a828d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActionStatus>)))
  "Returns full string definition for message of type '<ActionStatus>"
  (cl:format cl:nil "Header header~%bool result~%uint32 id~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActionStatus)))
  "Returns full string definition for message of type 'ActionStatus"
  (cl:format cl:nil "Header header~%bool result~%uint32 id~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActionStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActionStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'ActionStatus
    (cl:cons ':header (header msg))
    (cl:cons ':result (result msg))
    (cl:cons ':id (id msg))
))
