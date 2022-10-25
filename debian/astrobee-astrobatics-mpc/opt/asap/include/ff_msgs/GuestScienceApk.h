// Generated by gencpp from file ff_msgs/GuestScienceApk.msg
// DO NOT EDIT!


#ifndef FF_MSGS_MESSAGE_GUESTSCIENCEAPK_H
#define FF_MSGS_MESSAGE_GUESTSCIENCEAPK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <ff_msgs/GuestScienceCommand.h>

namespace ff_msgs
{
template <class ContainerAllocator>
struct GuestScienceApk_
{
  typedef GuestScienceApk_<ContainerAllocator> Type;

  GuestScienceApk_()
    : apk_name()
    , short_name()
    , primary(false)
    , commands()  {
    }
  GuestScienceApk_(const ContainerAllocator& _alloc)
    : apk_name(_alloc)
    , short_name(_alloc)
    , primary(false)
    , commands(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _apk_name_type;
  _apk_name_type apk_name;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _short_name_type;
  _short_name_type short_name;

   typedef uint8_t _primary_type;
  _primary_type primary;

   typedef std::vector< ::ff_msgs::GuestScienceCommand_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::ff_msgs::GuestScienceCommand_<ContainerAllocator> >::other >  _commands_type;
  _commands_type commands;





  typedef boost::shared_ptr< ::ff_msgs::GuestScienceApk_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ff_msgs::GuestScienceApk_<ContainerAllocator> const> ConstPtr;

}; // struct GuestScienceApk_

typedef ::ff_msgs::GuestScienceApk_<std::allocator<void> > GuestScienceApk;

typedef boost::shared_ptr< ::ff_msgs::GuestScienceApk > GuestScienceApkPtr;
typedef boost::shared_ptr< ::ff_msgs::GuestScienceApk const> GuestScienceApkConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ff_msgs::GuestScienceApk_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ff_msgs::GuestScienceApk_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::ff_msgs::GuestScienceApk_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ff_msgs::GuestScienceApk_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::GuestScienceApk_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::GuestScienceApk_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::GuestScienceApk_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::GuestScienceApk_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ff_msgs::GuestScienceApk_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8ed1d23e09733f18dbf96d2f9cd798e5";
  }

  static const char* value(const ::ff_msgs::GuestScienceApk_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8ed1d23e09733f18ULL;
  static const uint64_t static_value2 = 0xdbf96d2f9cd798e5ULL;
};

template<class ContainerAllocator>
struct DataType< ::ff_msgs::GuestScienceApk_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff_msgs/GuestScienceApk";
  }

  static const char* value(const ::ff_msgs::GuestScienceApk_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ff_msgs::GuestScienceApk_<ContainerAllocator> >
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
# Message used to contain information about a guest science apk\n\
\n\
# Full apk name\n\
string apk_name\n\
\n\
# Short (human readable) name of the apk\n\
string short_name\n\
\n\
# Whether the apk is primary or secondary\n\
bool primary\n\
\n\
# List of commands the apk will accept\n\
ff_msgs/GuestScienceCommand[] commands\n\
\n\
================================================================================\n\
MSG: ff_msgs/GuestScienceCommand\n\
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
# Message used to store guest science commands\n\
\n\
# Name of command\n\
string name\n\
\n\
# Syntax of the command\n\
string command\n\
";
  }

  static const char* value(const ::ff_msgs::GuestScienceApk_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ff_msgs::GuestScienceApk_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.apk_name);
      stream.next(m.short_name);
      stream.next(m.primary);
      stream.next(m.commands);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GuestScienceApk_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ff_msgs::GuestScienceApk_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ff_msgs::GuestScienceApk_<ContainerAllocator>& v)
  {
    s << indent << "apk_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.apk_name);
    s << indent << "short_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.short_name);
    s << indent << "primary: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.primary);
    s << indent << "commands[]" << std::endl;
    for (size_t i = 0; i < v.commands.size(); ++i)
    {
      s << indent << "  commands[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::ff_msgs::GuestScienceCommand_<ContainerAllocator> >::stream(s, indent + "    ", v.commands[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // FF_MSGS_MESSAGE_GUESTSCIENCEAPK_H
