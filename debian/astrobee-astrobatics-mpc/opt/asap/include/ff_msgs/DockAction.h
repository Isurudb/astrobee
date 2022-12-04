// Generated by gencpp from file ff_msgs/DockAction.msg
// DO NOT EDIT!


#ifndef FF_MSGS_MESSAGE_DOCKACTION_H
#define FF_MSGS_MESSAGE_DOCKACTION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <ff_msgs/DockActionGoal.h>
#include <ff_msgs/DockActionResult.h>
#include <ff_msgs/DockActionFeedback.h>

namespace ff_msgs
{
template <class ContainerAllocator>
struct DockAction_
{
  typedef DockAction_<ContainerAllocator> Type;

  DockAction_()
    : action_goal()
    , action_result()
    , action_feedback()  {
    }
  DockAction_(const ContainerAllocator& _alloc)
    : action_goal(_alloc)
    , action_result(_alloc)
    , action_feedback(_alloc)  {
  (void)_alloc;
    }



   typedef  ::ff_msgs::DockActionGoal_<ContainerAllocator>  _action_goal_type;
  _action_goal_type action_goal;

   typedef  ::ff_msgs::DockActionResult_<ContainerAllocator>  _action_result_type;
  _action_result_type action_result;

   typedef  ::ff_msgs::DockActionFeedback_<ContainerAllocator>  _action_feedback_type;
  _action_feedback_type action_feedback;





  typedef boost::shared_ptr< ::ff_msgs::DockAction_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ff_msgs::DockAction_<ContainerAllocator> const> ConstPtr;

}; // struct DockAction_

typedef ::ff_msgs::DockAction_<std::allocator<void> > DockAction;

typedef boost::shared_ptr< ::ff_msgs::DockAction > DockActionPtr;
typedef boost::shared_ptr< ::ff_msgs::DockAction const> DockActionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ff_msgs::DockAction_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ff_msgs::DockAction_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ff_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/home/isuru/arm_cross/rootfs/opt/ros/kinetic/share/std_msgs/cmake/../msg', '/home/isuru/arm_cross/rootfs/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/home/isuru/arm_cross/rootfs/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'trajectory_msgs': ['/home/isuru/arm_cross/rootfs/opt/ros/kinetic/share/trajectory_msgs/cmake/../msg'], 'sensor_msgs': ['/home/isuru/arm_cross/rootfs/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/home/isuru/arm_cross/rootfs/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'ff_msgs': ['/home/isuru/Forked_astrobee/astrobee/src/communications/ff_msgs/msg', '/home/isuru/Forked_astrobee/astrobee/debian/devel/.private/ff_msgs/share/ff_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ff_msgs::DockAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ff_msgs::DockAction_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::DockAction_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::DockAction_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::DockAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::DockAction_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ff_msgs::DockAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e5850d2946aabf5acd2b50c016179109";
  }

  static const char* value(const ::ff_msgs::DockAction_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe5850d2946aabf5aULL;
  static const uint64_t static_value2 = 0xcd2b50c016179109ULL;
};

template<class ContainerAllocator>
struct DataType< ::ff_msgs::DockAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff_msgs/DockAction";
  }

  static const char* value(const ::ff_msgs::DockAction_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ff_msgs::DockAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
DockActionGoal action_goal\n\
DockActionResult action_result\n\
DockActionFeedback action_feedback\n\
\n\
================================================================================\n\
MSG: ff_msgs/DockActionGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalID goal_id\n\
DockGoal goal\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalID\n\
# The stamp should store the time at which this goal was requested.\n\
# It is used by an action server when it tries to preempt all\n\
# goals that were requested before a certain time\n\
time stamp\n\
\n\
# The id provides a way to associate feedback and\n\
# result message with specific goal requests. The id\n\
# specified must be unique.\n\
string id\n\
\n\
\n\
================================================================================\n\
MSG: ff_msgs/DockGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# Copyright (c) 2017, United States Government, as represented by the\n\
# Administrator of the National Aeronautics and Space Administration.\n\
# \n\
# All rights reserved.\n\
# \n\
# The Astrobee platform is licensed under the Apache License, Version 2.0\n\
# (the \"License\"); you may not use this file except in compliance with the\n\
# License. You may obtain a copy of the License at\n\
# \n\
#     http://www.apache.org/licenses/LICENSE-2.0\n\
# \n\
# Unless required by applicable law or agreed to in writing, software\n\
# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT\n\
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the\n\
# License for the specific language governing permissions and limitations\n\
# under the License.\n\
#\n\
# Dock action for the Docking node\n\
\n\
# Do we want to dock or undock\n\
uint8 command\n\
uint8 DOCK    = 1\n\
uint8 UNDOCK  = 2\n\
\n\
# Which berth (values must match)\n\
uint8 berth\n\
uint8 BERTH_UNKNOWN = 0 # When we undock, we don't know what berth we are in\n\
uint8 BERTH_1       = 1\n\
uint8 BERTH_2       = 2\n\
\n\
# Return to dock\n\
bool return_dock\n\
\n\
\n\
================================================================================\n\
MSG: ff_msgs/DockActionResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
DockResult result\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalStatus\n\
GoalID goal_id\n\
uint8 status\n\
uint8 PENDING         = 0   # The goal has yet to be processed by the action server\n\
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server\n\
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing\n\
                            #   and has since completed its execution (Terminal State)\n\
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)\n\
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due\n\
                            #    to some failure (Terminal State)\n\
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,\n\
                            #    because the goal was unattainable or invalid (Terminal State)\n\
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing\n\
                            #    and has not yet completed execution\n\
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,\n\
                            #    but the action server has not yet confirmed that the goal is canceled\n\
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing\n\
                            #    and was successfully cancelled (Terminal State)\n\
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be\n\
                            #    sent over the wire by an action server\n\
\n\
#Allow for the user to associate a string with GoalStatus for debugging\n\
string text\n\
\n\
\n\
================================================================================\n\
MSG: ff_msgs/DockResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
# Result\n\
int32 response\n\
int32 CANCELLED                          = 5\n\
int32 ALREADY_DOCKED                     = 4\n\
int32 ALREADY_UNDOCKED                   = 3\n\
int32 UNDOCKED                           = 2\n\
int32 DOCKED                             = 1\n\
int32 PREEMPTED                          = 0\n\
int32 INVALID_COMMAND                    = -1\n\
int32 INVALID_BERTH                      = -2\n\
int32 NOT_IN_UNDOCKED_STATE              = -3\n\
int32 NOT_IN_DOCKED_STATE                = -4\n\
int32 SWITCH_TO_ML_FAILED                = -5\n\
int32 SWITCH_TO_AR_FAILED                = -6\n\
int32 SWITCH_TO_NO_FAILED                = -7\n\
int32 PREP_DISABLE_FAILED                = -8\n\
int32 PREP_ENABLE_FAILED                 = -9\n\
int32 MOTION_APPROACH_FAILED             = -10\n\
int32 MOTION_COMPLETE_FAILED             = -11\n\
int32 MOTION_ATTACHED_FAILED             = -12\n\
int32 EPS_UNDOCK_FAILED                  = -13\n\
int32 EPS_DOCK_FAILED                    = -14\n\
int32 TOO_FAR_AWAY_FROM_APPROACH         = -15\n\
\n\
# Human readable FSM result for debugging\n\
string fsm_result\n\
\n\
\n\
================================================================================\n\
MSG: ff_msgs/DockActionFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
DockFeedback feedback\n\
\n\
================================================================================\n\
MSG: ff_msgs/DockFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
# Feedback\n\
ff_msgs/DockState state\n\
\n\
\n\
================================================================================\n\
MSG: ff_msgs/DockState\n\
# Copyright (c) 2017, United States Government, as represented by the\n\
# Administrator of the National Aeronautics and Space Administration.\n\
# \n\
# All rights reserved.\n\
# \n\
# The Astrobee platform is licensed under the Apache License, Version 2.0\n\
# (the \"License\"); you may not use this file except in compliance with the\n\
# License. You may obtain a copy of the License at\n\
# \n\
#     http://www.apache.org/licenses/LICENSE-2.0\n\
# \n\
# Unless required by applicable law or agreed to in writing, software\n\
# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT\n\
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the\n\
# License for the specific language governing permissions and limitations\n\
# under the License.\n\
#\n\
# Response for Dock/Undock goals\n\
\n\
# Header with timestamp\n\
std_msgs/Header header\n\
\n\
# Feedback\n\
int8 state\n\
int8 RECOVERY_SWITCHING_TO_ML_LOC       = 15\n\
int8 RECOVERY_MOVING_TO_APPROACH_POSE   = 14\n\
int8 RECOVERY_WAITING_FOR_SPIN_DOWN     = 13\n\
int8 RECOVERY_SWITCHING_TO_NO_LOC       = 12\n\
int8 INITIALIZING                       = 11\n\
int8 UNKNOWN                            = 10\n\
int8 DOCKING_MAX_STATE                  = 7\n\
int8 DOCKING_SWITCHING_TO_ML_LOC        = 7\n\
int8 DOCKING_MOVING_TO_APPROACH_POSE    = 6\n\
int8 DOCKING_SWITCHING_TO_AR_LOC        = 5\n\
int8 DOCKING_MOVING_TO_COMPLETE_POSE    = 4\n\
int8 DOCKING_CHECKING_ATTACHED          = 3\n\
int8 DOCKING_WAITING_FOR_SPIN_DOWN      = 2\n\
int8 DOCKING_SWITCHING_TO_NO_LOC        = 1\n\
int8 DOCKED                             = 0\n\
int8 UNDOCKING_SWITCHING_TO_ML_LOC      = -1\n\
int8 UNDOCKING_WAITING_FOR_SPIN_UP      = -2\n\
int8 UNDOCKING_MOVING_TO_APPROACH_POSE  = -3\n\
int8 UNDOCKED                           = -4\n\
int8 UNDOCKING_MAX_STATE                = -4\n\
\n\
# A human readble version of the (event) -> [state] transition\n\
string fsm_event\n\
string fsm_state\n\
";
  }

  static const char* value(const ::ff_msgs::DockAction_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ff_msgs::DockAction_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.action_goal);
      stream.next(m.action_result);
      stream.next(m.action_feedback);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DockAction_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ff_msgs::DockAction_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ff_msgs::DockAction_<ContainerAllocator>& v)
  {
    s << indent << "action_goal: ";
    s << std::endl;
    Printer< ::ff_msgs::DockActionGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.action_goal);
    s << indent << "action_result: ";
    s << std::endl;
    Printer< ::ff_msgs::DockActionResult_<ContainerAllocator> >::stream(s, indent + "  ", v.action_result);
    s << indent << "action_feedback: ";
    s << std::endl;
    Printer< ::ff_msgs::DockActionFeedback_<ContainerAllocator> >::stream(s, indent + "  ", v.action_feedback);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FF_MSGS_MESSAGE_DOCKACTION_H