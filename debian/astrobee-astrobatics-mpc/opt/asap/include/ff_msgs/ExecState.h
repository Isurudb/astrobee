// Generated by gencpp from file ff_msgs/ExecState.msg
// DO NOT EDIT!


#ifndef FF_MSGS_MESSAGE_EXECSTATE_H
#define FF_MSGS_MESSAGE_EXECSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ff_msgs
{
template <class ContainerAllocator>
struct ExecState_
{
  typedef ExecState_<ContainerAllocator> Type;

  ExecState_()
    : state(0)  {
    }
  ExecState_(const ContainerAllocator& _alloc)
    : state(0)  {
  (void)_alloc;
    }



   typedef uint8_t _state_type;
  _state_type state;



  enum {
    IDLE = 0u,
    EXECUTING = 1u,
    PAUSED = 2u,
    ERROR = 3u,
  };


  typedef boost::shared_ptr< ::ff_msgs::ExecState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ff_msgs::ExecState_<ContainerAllocator> const> ConstPtr;

}; // struct ExecState_

typedef ::ff_msgs::ExecState_<std::allocator<void> > ExecState;

typedef boost::shared_ptr< ::ff_msgs::ExecState > ExecStatePtr;
typedef boost::shared_ptr< ::ff_msgs::ExecState const> ExecStateConstPtr;

// constants requiring out of line definition

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ff_msgs::ExecState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ff_msgs::ExecState_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ff_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/home/isuru/arm_cross/rootfs/opt/ros/kinetic/share/std_msgs/cmake/../msg', '/home/isuru/arm_cross/rootfs/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/home/isuru/arm_cross/rootfs/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'trajectory_msgs': ['/home/isuru/arm_cross/rootfs/opt/ros/kinetic/share/trajectory_msgs/cmake/../msg'], 'sensor_msgs': ['/home/isuru/arm_cross/rootfs/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/home/isuru/arm_cross/rootfs/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'ff_msgs': ['/home/isuru/Forked_astrobee/astrobee/src/communications/ff_msgs/msg', '/home/isuru/Forked_astrobee/astrobee/debian/devel/.private/ff_msgs/share/ff_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ff_msgs::ExecState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ff_msgs::ExecState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::ExecState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::ExecState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::ExecState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::ExecState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ff_msgs::ExecState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "10a48ab48fd2106828caec7c2cbb9e91";
  }

  static const char* value(const ::ff_msgs::ExecState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x10a48ab48fd21068ULL;
  static const uint64_t static_value2 = 0x28caec7c2cbb9e91ULL;
};

template<class ContainerAllocator>
struct DataType< ::ff_msgs::ExecState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff_msgs/ExecState";
  }

  static const char* value(const ::ff_msgs::ExecState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ff_msgs::ExecState_<ContainerAllocator> >
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
# Execution States, based off of the enumeration constants in\n\
# rapid::ext::astrobee::AgentState\n\
#\n\
# *MUST* be kept in sync with the DDS IDL file in astrobee_common\n\
\n\
uint8 IDLE      = 0   # Process is idle\n\
uint8 EXECUTING = 1   # Process is executing\n\
uint8 PAUSED    = 2   # Process is paused\n\
uint8 ERROR     = 3   # Process encountered an error\n\
\n\
# Execution state\n\
uint8 state\n\
";
  }

  static const char* value(const ::ff_msgs::ExecState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ff_msgs::ExecState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ExecState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ff_msgs::ExecState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ff_msgs::ExecState_<ContainerAllocator>& v)
  {
    s << indent << "state: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FF_MSGS_MESSAGE_EXECSTATE_H
