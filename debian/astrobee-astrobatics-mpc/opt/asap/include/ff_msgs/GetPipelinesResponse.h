// Generated by gencpp from file ff_msgs/GetPipelinesResponse.msg
// DO NOT EDIT!


#ifndef FF_MSGS_MESSAGE_GETPIPELINESRESPONSE_H
#define FF_MSGS_MESSAGE_GETPIPELINESRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <ff_msgs/LocalizationPipeline.h>

namespace ff_msgs
{
template <class ContainerAllocator>
struct GetPipelinesResponse_
{
  typedef GetPipelinesResponse_<ContainerAllocator> Type;

  GetPipelinesResponse_()
    : pipelines()  {
    }
  GetPipelinesResponse_(const ContainerAllocator& _alloc)
    : pipelines(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::ff_msgs::LocalizationPipeline_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::ff_msgs::LocalizationPipeline_<ContainerAllocator> >::other >  _pipelines_type;
  _pipelines_type pipelines;





  typedef boost::shared_ptr< ::ff_msgs::GetPipelinesResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ff_msgs::GetPipelinesResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetPipelinesResponse_

typedef ::ff_msgs::GetPipelinesResponse_<std::allocator<void> > GetPipelinesResponse;

typedef boost::shared_ptr< ::ff_msgs::GetPipelinesResponse > GetPipelinesResponsePtr;
typedef boost::shared_ptr< ::ff_msgs::GetPipelinesResponse const> GetPipelinesResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ff_msgs::GetPipelinesResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ff_msgs::GetPipelinesResponse_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::ff_msgs::GetPipelinesResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ff_msgs::GetPipelinesResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::GetPipelinesResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::GetPipelinesResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::GetPipelinesResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::GetPipelinesResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ff_msgs::GetPipelinesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4fb31d141d0f152e9301905ffcaa8f48";
  }

  static const char* value(const ::ff_msgs::GetPipelinesResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4fb31d141d0f152eULL;
  static const uint64_t static_value2 = 0x9301905ffcaa8f48ULL;
};

template<class ContainerAllocator>
struct DataType< ::ff_msgs::GetPipelinesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff_msgs/GetPipelinesResponse";
  }

  static const char* value(const ::ff_msgs::GetPipelinesResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ff_msgs::GetPipelinesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
ff_msgs/LocalizationPipeline[] pipelines\n\
\n\
\n\
================================================================================\n\
MSG: ff_msgs/LocalizationPipeline\n\
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
# Information about a pipeline\n\
\n\
string id                     # Short id for the pipeline\n\
uint8 mode                    # EKF mode for the pipeline\n\
string name                   # Long name for the pipe\n\
bool requires_filter          # Does this pipeline require the EKF\n\
bool requires_optical_flow    # Does this pipeline require optical flow\n\
";
  }

  static const char* value(const ::ff_msgs::GetPipelinesResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ff_msgs::GetPipelinesResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pipelines);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetPipelinesResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ff_msgs::GetPipelinesResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ff_msgs::GetPipelinesResponse_<ContainerAllocator>& v)
  {
    s << indent << "pipelines[]" << std::endl;
    for (size_t i = 0; i < v.pipelines.size(); ++i)
    {
      s << indent << "  pipelines[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::ff_msgs::LocalizationPipeline_<ContainerAllocator> >::stream(s, indent + "    ", v.pipelines[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // FF_MSGS_MESSAGE_GETPIPELINESRESPONSE_H
