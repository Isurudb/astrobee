// Generated by gencpp from file ff_msgs/DataToDiskState.msg
// DO NOT EDIT!


#ifndef FF_MSGS_MESSAGE_DATATODISKSTATE_H
#define FF_MSGS_MESSAGE_DATATODISKSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <ff_msgs/SaveSettings.h>

namespace ff_msgs
{
template <class ContainerAllocator>
struct DataToDiskState_
{
  typedef DataToDiskState_<ContainerAllocator> Type;

  DataToDiskState_()
    : header()
    , name()
    , recording(false)
    , topic_save_settings()  {
    }
  DataToDiskState_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , name(_alloc)
    , recording(false)
    , topic_save_settings(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  _name_type name;

   typedef uint8_t _recording_type;
  _recording_type recording;

   typedef std::vector< ::ff_msgs::SaveSettings_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::ff_msgs::SaveSettings_<ContainerAllocator> >::other >  _topic_save_settings_type;
  _topic_save_settings_type topic_save_settings;





  typedef boost::shared_ptr< ::ff_msgs::DataToDiskState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ff_msgs::DataToDiskState_<ContainerAllocator> const> ConstPtr;

}; // struct DataToDiskState_

typedef ::ff_msgs::DataToDiskState_<std::allocator<void> > DataToDiskState;

typedef boost::shared_ptr< ::ff_msgs::DataToDiskState > DataToDiskStatePtr;
typedef boost::shared_ptr< ::ff_msgs::DataToDiskState const> DataToDiskStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ff_msgs::DataToDiskState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ff_msgs::DataToDiskState_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::ff_msgs::DataToDiskState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ff_msgs::DataToDiskState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::DataToDiskState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::DataToDiskState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::DataToDiskState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::DataToDiskState_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ff_msgs::DataToDiskState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "68d7ec16d4c7bc2b6e1a00776a76b4f7";
  }

  static const char* value(const ::ff_msgs::DataToDiskState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x68d7ec16d4c7bc2bULL;
  static const uint64_t static_value2 = 0x6e1a00776a76b4f7ULL;
};

template<class ContainerAllocator>
struct DataType< ::ff_msgs::DataToDiskState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff_msgs/DataToDiskState";
  }

  static const char* value(const ::ff_msgs::DataToDiskState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ff_msgs::DataToDiskState_<ContainerAllocator> >
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
\n\
# Data to disk state message used to let ground operators know which topics\n\
# are currently being recorded.\n\
\n\
# Header with timestamp\n\
std_msgs/Header header\n\
\n\
# Name of the latest data to disk file uploaded from the ground\n\
string name\n\
\n\
# Whether the data bagger is recording a bag or not\n\
bool recording\n\
\n\
# An array containing information about the topics being recorded\n\
ff_msgs/SaveSettings[] topic_save_settings\n\
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
MSG: ff_msgs/SaveSettings\n\
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
\n\
# The save settings message contains information about the topics currently\n\
# being recorded.\n\
\n\
# Name of topic\n\
string topic_name\n\
\n\
# Topic saved to disk; upon docking it is downlinked\n\
uint8 IMMEDIATE   = 0\n\
\n\
# Topic saved to disk; upon docking it is transferred to ISS server for later\n\
# downlink\n\
uint8 DELAYED     = 1\n\
\n\
# Downlink option indicates if and when the data in the rostopic is downlinked\n\
uint8 downlinkOption\n\
\n\
# Times per second to save the data (Hz)\n\
float32 frequency\n\
";
  }

  static const char* value(const ::ff_msgs::DataToDiskState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ff_msgs::DataToDiskState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.name);
      stream.next(m.recording);
      stream.next(m.topic_save_settings);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DataToDiskState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ff_msgs::DataToDiskState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ff_msgs::DataToDiskState_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "recording: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.recording);
    s << indent << "topic_save_settings[]" << std::endl;
    for (size_t i = 0; i < v.topic_save_settings.size(); ++i)
    {
      s << indent << "  topic_save_settings[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::ff_msgs::SaveSettings_<ContainerAllocator> >::stream(s, indent + "    ", v.topic_save_settings[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // FF_MSGS_MESSAGE_DATATODISKSTATE_H