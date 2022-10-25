// Generated by gencpp from file ff_msgs/ArmStateStamped.msg
// DO NOT EDIT!


#ifndef FF_MSGS_MESSAGE_ARMSTATESTAMPED_H
#define FF_MSGS_MESSAGE_ARMSTATESTAMPED_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <ff_msgs/ArmJointState.h>
#include <ff_msgs/ArmGripperState.h>

namespace ff_msgs
{
template <class ContainerAllocator>
struct ArmStateStamped_
{
  typedef ArmStateStamped_<ContainerAllocator> Type;

  ArmStateStamped_()
    : header()
    , joint_state()
    , gripper_state()  {
    }
  ArmStateStamped_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , joint_state(_alloc)
    , gripper_state(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::ff_msgs::ArmJointState_<ContainerAllocator>  _joint_state_type;
  _joint_state_type joint_state;

   typedef  ::ff_msgs::ArmGripperState_<ContainerAllocator>  _gripper_state_type;
  _gripper_state_type gripper_state;





  typedef boost::shared_ptr< ::ff_msgs::ArmStateStamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ff_msgs::ArmStateStamped_<ContainerAllocator> const> ConstPtr;

}; // struct ArmStateStamped_

typedef ::ff_msgs::ArmStateStamped_<std::allocator<void> > ArmStateStamped;

typedef boost::shared_ptr< ::ff_msgs::ArmStateStamped > ArmStateStampedPtr;
typedef boost::shared_ptr< ::ff_msgs::ArmStateStamped const> ArmStateStampedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ff_msgs::ArmStateStamped_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ff_msgs::ArmStateStamped_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ff_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/home/isuru/arm_cross/rootfs/opt/ros/kinetic/share/std_msgs/cmake/../msg', '/home/isuru/arm_cross/rootfs/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/home/isuru/arm_cross/rootfs/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'trajectory_msgs': ['/home/isuru/arm_cross/rootfs/opt/ros/kinetic/share/trajectory_msgs/cmake/../msg'], 'sensor_msgs': ['/home/isuru/arm_cross/rootfs/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/home/isuru/arm_cross/rootfs/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'ff_msgs': ['/home/isuru/Forked_astrobee/astrobee/src/communications/ff_msgs/msg', '/home/isuru/Forked_astrobee/astrobee/debian/devel/.private/ff_msgs/share/ff_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ff_msgs::ArmStateStamped_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ff_msgs::ArmStateStamped_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::ArmStateStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::ArmStateStamped_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::ArmStateStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::ArmStateStamped_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ff_msgs::ArmStateStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3861c96e90f30d3bd53dc5e09edfb937";
  }

  static const char* value(const ::ff_msgs::ArmStateStamped_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3861c96e90f30d3bULL;
  static const uint64_t static_value2 = 0xd53dc5e09edfb937ULL;
};

template<class ContainerAllocator>
struct DataType< ::ff_msgs::ArmStateStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff_msgs/ArmStateStamped";
  }

  static const char* value(const ::ff_msgs::ArmStateStamped_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ff_msgs::ArmStateStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Copyright (c) 2017, United States Government, as represented by the\n\
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
# ArmState message\n\
#\n\
# *MUST* be kept in sync with rapid::ext::astrobee::ArmState\n\
\n\
std_msgs/Header header\n\
\n\
ff_msgs/ArmJointState joint_state\n\
ff_msgs/ArmGripperState  gripper_state\n\
\n\
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
MSG: ff_msgs/ArmJointState\n\
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
# Arm Joint State enum.\n\
#\n\
# *MUST* be kept in sync with rapid::ext::astrobee::ArmState\n\
\n\
uint8 UNKNOWN   = 0\n\
uint8 STOWED    = 1\n\
uint8 DEPLOYING = 2\n\
uint8 STOPPED   = 3\n\
uint8 MOVING    = 4\n\
uint8 STOWING   = 5\n\
\n\
uint8 state\n\
\n\
================================================================================\n\
MSG: ff_msgs/ArmGripperState\n\
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
# Arm Gripper State enum\n\
#\n\
# *MUST* be kept in sync with rapid::ext::astrobee::ArmState\n\
\n\
uint8 UNKNOWN      = 0\n\
uint8 UNCALIBRATED = 1\n\
uint8 CALIBRATING  = 2\n\
uint8 CLOSED       = 3\n\
uint8 OPEN         = 4\n\
\n\
uint8 state\n\
";
  }

  static const char* value(const ::ff_msgs::ArmStateStamped_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ff_msgs::ArmStateStamped_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.joint_state);
      stream.next(m.gripper_state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ArmStateStamped_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ff_msgs::ArmStateStamped_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ff_msgs::ArmStateStamped_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "joint_state: ";
    s << std::endl;
    Printer< ::ff_msgs::ArmJointState_<ContainerAllocator> >::stream(s, indent + "  ", v.joint_state);
    s << indent << "gripper_state: ";
    s << std::endl;
    Printer< ::ff_msgs::ArmGripperState_<ContainerAllocator> >::stream(s, indent + "  ", v.gripper_state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FF_MSGS_MESSAGE_ARMSTATESTAMPED_H
