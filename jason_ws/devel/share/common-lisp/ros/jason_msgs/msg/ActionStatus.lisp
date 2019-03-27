; Auto-generated. Do not edit!


(cl:in-package jason_msgs-msg)


;//! \htmlinclude ActionStatus.msg.html

(cl:defclass <ActionStatus> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ActionStatus (<ActionStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActionStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActionStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jason_msgs-msg:<ActionStatus> is deprecated: use jason_msgs-msg:ActionStatus instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActionStatus>) ostream)
  "Serializes a message object of type '<ActionStatus>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActionStatus>) istream)
  "Deserializes a message object of type '<ActionStatus>"
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
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActionStatus)))
  "Returns md5sum for a message object of type 'ActionStatus"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActionStatus>)))
  "Returns full string definition for message of type '<ActionStatus>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActionStatus)))
  "Returns full string definition for message of type 'ActionStatus"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActionStatus>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActionStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'ActionStatus
))
