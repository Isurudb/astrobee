// Generated by gencpp from file ff_msgs/FaultInfo.msg
// DO NOT EDIT!


#ifndef FF_MSGS_MESSAGE_FAULTINFO_H
#define FF_MSGS_MESSAGE_FAULTINFO_H


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
struct FaultInfo_
{
  typedef FaultInfo_<ContainerAllocator> Type;

  FaultInfo_()
    : subsystem(0)
    , node(0)
    , id(0)
    , warning(false)
    , description()  {
    }
  FaultInfo_(const ContainerAllocator& _alloc)
    : subsystem(0)
    , node(0)
    , id(0)
    , warning(false)
    , description(_alloc)  {
  (void)_alloc;
    }



   typedef uint16_t _subsystem_type;
  _subsystem_type subsystem;

   typedef uint16_t _node_type;
  _node_type node;

   typedef uint32_t _id_type;
  _id_type id;

   typedef uint8_t _warning_type;
  _warning_type warning;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _description_type;
  _description_type description;





  typedef boost::shared_ptr< ::ff_msgs::FaultInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ff_msgs::FaultInfo_<ContainerAllocator> const> ConstPtr;

}; // struct FaultInfo_

typedef ::ff_msgs::FaultInfo_<std::allocator<void> > FaultInfo;

typedef boost::shared_ptr< ::ff_msgs::FaultInfo > FaultInfoPtr;
typedef boost::shared_ptr< ::ff_msgs::FaultInfo const> FaultInfoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ff_msgs::FaultInfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ff_msgs::FaultInfo_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::ff_msgs::FaultInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ff_msgs::FaultInfo_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::FaultInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::FaultInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::FaultInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::FaultInfo_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ff_msgs::FaultInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1f6014a9106a0f40b77f475f6f9592fa";
  }

  static const char* value(const ::ff_msgs::FaultInfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1f6014a9106a0f40ULL;
  static const uint64_t static_value2 = 0xb77f475f6f9592faULL;
};

template<class ContainerAllocator>
struct DataType< ::ff_msgs::FaultInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff_msgs/FaultInfo";
  }

  static const char* value(const ::ff_msgs::FaultInfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ff_msgs::FaultInfo_<ContainerAllocator> >
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
# Fault info message is used in the fault config message to contain all the \n\
# information GDS needs to know about a fault\n\
\n\
uint16 subsystem    # index into subsystem names array found in fault config msg\n\
\n\
uint16 node         # index into node names array found in fault config msg\n\
\n\
uint32 id           # id corresponding to the fault\n\
\n\
bool warning        # whether the fault is a warning or not\n\
\n\
string description  # A short description of why the fault occurred\n\
";
  }

  static const char* value(const ::ff_msgs::FaultInfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ff_msgs::FaultInfo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.subsystem);
      stream.next(m.node);
      stream.next(m.id);
      stream.next(m.warning);
      stream.next(m.description);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FaultInfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ff_msgs::FaultInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ff_msgs::FaultInfo_<ContainerAllocator>& v)
  {
    s << indent << "subsystem: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.subsystem);
    s << indent << "node: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.node);
    s << indent << "id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.id);
    s << indent << "warning: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.warning);
    s << indent << "description: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.description);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FF_MSGS_MESSAGE_FAULTINFO_H