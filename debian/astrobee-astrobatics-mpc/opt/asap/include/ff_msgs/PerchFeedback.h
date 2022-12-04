// Generated by gencpp from file ff_msgs/PerchFeedback.msg
// DO NOT EDIT!


#ifndef FF_MSGS_MESSAGE_PERCHFEEDBACK_H
#define FF_MSGS_MESSAGE_PERCHFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <ff_msgs/PerchState.h>

namespace ff_msgs
{
template <class ContainerAllocator>
struct PerchFeedback_
{
  typedef PerchFeedback_<ContainerAllocator> Type;

  PerchFeedback_()
    : state()  {
    }
  PerchFeedback_(const ContainerAllocator& _alloc)
    : state(_alloc)  {
  (void)_alloc;
    }



   typedef  ::ff_msgs::PerchState_<ContainerAllocator>  _state_type;
  _state_type state;





  typedef boost::shared_ptr< ::ff_msgs::PerchFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ff_msgs::PerchFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct PerchFeedback_

typedef ::ff_msgs::PerchFeedback_<std::allocator<void> > PerchFeedback;

typedef boost::shared_ptr< ::ff_msgs::PerchFeedback > PerchFeedbackPtr;
typedef boost::shared_ptr< ::ff_msgs::PerchFeedback const> PerchFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ff_msgs::PerchFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ff_msgs::PerchFeedback_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::ff_msgs::PerchFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ff_msgs::PerchFeedback_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::PerchFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::PerchFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::PerchFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::PerchFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ff_msgs::PerchFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ce91b2c26500c663aeec651c3bf03ceb";
  }

  static const char* value(const ::ff_msgs::PerchFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xce91b2c26500c663ULL;
  static const uint64_t static_value2 = 0xaeec651c3bf03cebULL;
};

template<class ContainerAllocator>
struct DataType< ::ff_msgs::PerchFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff_msgs/PerchFeedback";
  }

  static const char* value(const ::ff_msgs::PerchFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ff_msgs::PerchFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
# Feedback\n\
ff_msgs/PerchState state\n\
\n\
\n\
================================================================================\n\
MSG: ff_msgs/PerchState\n\
# Copyright (c) 2017, United States Government, as represented by the\n\
# Administrator of the National Aeronautics and Space Administration.\n\
#\n\
# All rights reserved.\n\
#\n\
# The Astrobee platform is licensed under the Apache License, Version 2.0\n\
# (the \"License\"); you may not use this file except in compliance with the\n\
# License. You may obtain a copy of the License at\n\
#\n\
#     http://www.apache.org/licenses/LICENSE-2.0\n\
#\n\
# Unless required by applicable law or agreed to in writing, software\n\
# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT\n\
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the\n\
# License for the specific language governing permissions and limitations\n\
# under the License.\n\
#\n\
# The state of the perching system\n\
\n\
# Header with timestamp\n\
std_msgs/Header header\n\
\n\
# Feedback\n\
int8 state\n\
\n\
int8 RECOVERY_MOVING_TO_RECOVERY_POSE   = 18\n\
int8 RECOVERY_SWITCHING_TO_ML_LOC       = 17\n\
int8 RECOVERY_STOWING_ARM               = 16\n\
int8 RECOVERY_MOVING_TO_APPROACH_POSE   = 15\n\
int8 RECOVERY_OPENING_GRIPPER           = 14\n\
int8 INITIALIZING                       = 13\n\
int8 UNKNOWN                            = 12\n\
# Used to check the perching/unperching ranges\n\
int8 PERCHING_MAX_STATE                 = 11\n\
int8 PERCHING_SWITCHING_TO_HR_LOC       = 11\n\
int8 PERCHING_MOVING_TO_APPROACH_POSE   = 10\n\
int8 PERCHING_ENSURING_APPROACH_POSE    = 9\n\
int8 PERCHING_DEPLOYING_ARM             = 8\n\
int8 PERCHING_OPENING_GRIPPER           = 7\n\
int8 PERCHING_MOVING_TO_COMPLETE_POSE   = 6\n\
int8 PERCHING_CLOSING_GRIPPER           = 5\n\
int8 PERCHING_CHECKING_ATTACHED         = 4\n\
int8 PERCHING_WAITING_FOR_SPIN_DOWN     = 3\n\
int8 PERCHING_SWITCHING_TO_PL_LOC       = 2\n\
int8 PERCHING_STOPPING                  = 1\n\
int8 PERCHED                            = 0\n\
int8 UNPERCHING_SWITCHING_TO_HR_LOC     = -1\n\
int8 UNPERCHING_WAITING_FOR_SPIN_UP     = -2\n\
int8 UNPERCHING_OPENING_GRIPPER         = -3\n\
int8 UNPERCHING_MOVING_TO_APPROACH_POSE = -4\n\
int8 UNPERCHING_STOWING_ARM             = -5\n\
int8 UNPERCHING_SWITCHING_TO_ML_LOC     = -6\n\
int8 UNPERCHED                          = -7\n\
# Used to check the perching/unperching ranges\n\
int8 UNPERCHING_MAX_STATE               = -7\n\
\n\
# A human readable version of the (event) -> [state] transition\n\
string fsm_event\n\
string fsm_state\n\
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
";
  }

  static const char* value(const ::ff_msgs::PerchFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ff_msgs::PerchFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PerchFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ff_msgs::PerchFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ff_msgs::PerchFeedback_<ContainerAllocator>& v)
  {
    s << indent << "state: ";
    s << std::endl;
    Printer< ::ff_msgs::PerchState_<ContainerAllocator> >::stream(s, indent + "  ", v.state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FF_MSGS_MESSAGE_PERCHFEEDBACK_H