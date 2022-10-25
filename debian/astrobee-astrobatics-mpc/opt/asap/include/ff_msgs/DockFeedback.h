// Generated by gencpp from file ff_msgs/DockFeedback.msg
// DO NOT EDIT!


#ifndef FF_MSGS_MESSAGE_DOCKFEEDBACK_H
#define FF_MSGS_MESSAGE_DOCKFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <ff_msgs/DockState.h>

namespace ff_msgs
{
template <class ContainerAllocator>
struct DockFeedback_
{
  typedef DockFeedback_<ContainerAllocator> Type;

  DockFeedback_()
    : state()  {
    }
  DockFeedback_(const ContainerAllocator& _alloc)
    : state(_alloc)  {
  (void)_alloc;
    }



   typedef  ::ff_msgs::DockState_<ContainerAllocator>  _state_type;
  _state_type state;





  typedef boost::shared_ptr< ::ff_msgs::DockFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ff_msgs::DockFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct DockFeedback_

typedef ::ff_msgs::DockFeedback_<std::allocator<void> > DockFeedback;

typedef boost::shared_ptr< ::ff_msgs::DockFeedback > DockFeedbackPtr;
typedef boost::shared_ptr< ::ff_msgs::DockFeedback const> DockFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ff_msgs::DockFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ff_msgs::DockFeedback_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::ff_msgs::DockFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ff_msgs::DockFeedback_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::DockFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::DockFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::DockFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::DockFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ff_msgs::DockFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ab5cbe2d052a7927a3632f11a64bfa27";
  }

  static const char* value(const ::ff_msgs::DockFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xab5cbe2d052a7927ULL;
  static const uint64_t static_value2 = 0xa3632f11a64bfa27ULL;
};

template<class ContainerAllocator>
struct DataType< ::ff_msgs::DockFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff_msgs/DockFeedback";
  }

  static const char* value(const ::ff_msgs::DockFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ff_msgs::DockFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
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
";
  }

  static const char* value(const ::ff_msgs::DockFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ff_msgs::DockFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DockFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ff_msgs::DockFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ff_msgs::DockFeedback_<ContainerAllocator>& v)
  {
    s << indent << "state: ";
    s << std::endl;
    Printer< ::ff_msgs::DockState_<ContainerAllocator> >::stream(s, indent + "  ", v.state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FF_MSGS_MESSAGE_DOCKFEEDBACK_H
