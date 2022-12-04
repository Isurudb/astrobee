// Generated by gencpp from file ff_msgs/VisualeyezCalibration.msg
// DO NOT EDIT!


#ifndef FF_MSGS_MESSAGE_VISUALEYEZCALIBRATION_H
#define FF_MSGS_MESSAGE_VISUALEYEZCALIBRATION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <ff_msgs/VisualeyezDataArray.h>

namespace ff_msgs
{
template <class ContainerAllocator>
struct VisualeyezCalibration_
{
  typedef VisualeyezCalibration_<ContainerAllocator> Type;

  VisualeyezCalibration_()
    : targets()  {
    }
  VisualeyezCalibration_(const ContainerAllocator& _alloc)
    : targets(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::ff_msgs::VisualeyezDataArray_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::ff_msgs::VisualeyezDataArray_<ContainerAllocator> >::other >  _targets_type;
  _targets_type targets;





  typedef boost::shared_ptr< ::ff_msgs::VisualeyezCalibration_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ff_msgs::VisualeyezCalibration_<ContainerAllocator> const> ConstPtr;

}; // struct VisualeyezCalibration_

typedef ::ff_msgs::VisualeyezCalibration_<std::allocator<void> > VisualeyezCalibration;

typedef boost::shared_ptr< ::ff_msgs::VisualeyezCalibration > VisualeyezCalibrationPtr;
typedef boost::shared_ptr< ::ff_msgs::VisualeyezCalibration const> VisualeyezCalibrationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ff_msgs::VisualeyezCalibration_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ff_msgs::VisualeyezCalibration_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::ff_msgs::VisualeyezCalibration_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ff_msgs::VisualeyezCalibration_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::VisualeyezCalibration_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::VisualeyezCalibration_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::VisualeyezCalibration_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::VisualeyezCalibration_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ff_msgs::VisualeyezCalibration_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a1e3f6982eba5d7a0edabf2dea3facb0";
  }

  static const char* value(const ::ff_msgs::VisualeyezCalibration_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa1e3f6982eba5d7aULL;
  static const uint64_t static_value2 = 0x0edabf2dea3facb0ULL;
};

template<class ContainerAllocator>
struct DataType< ::ff_msgs::VisualeyezCalibration_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff_msgs/VisualeyezCalibration";
  }

  static const char* value(const ::ff_msgs::VisualeyezCalibration_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ff_msgs::VisualeyezCalibration_<ContainerAllocator> >
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
# This message serializes the calibration data, so that we can write the results\n\
# To a calibration file, for loading later on.\n\
\n\
ff_msgs/VisualeyezDataArray[] targets   # List of targets\n\
\n\
================================================================================\n\
MSG: ff_msgs/VisualeyezDataArray\n\
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
# Raw Visualeyez data array and timestamp.\n\
\n\
Header header                           # Header with timestamp\n\
ff_msgs/VisualeyezData[] measurements   # List of all measurements\n\
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
MSG: ff_msgs/VisualeyezData\n\
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
# Raw Visualeyez data.\n\
\n\
uint8 tcmid                         # Transmission control module ID\n\
uint8 ledid                         # Light emitting diode ID\n\
geometry_msgs/Vector3 position      # Coordinate \n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
# It is only meant to represent a direction. Therefore, it does not\n\
# make sense to apply a translation to it (e.g., when applying a \n\
# generic rigid transformation to a Vector3, tf2 will only apply the\n\
# rotation). If you want your data to be translatable too, use the\n\
# geometry_msgs/Point message instead.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::ff_msgs::VisualeyezCalibration_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ff_msgs::VisualeyezCalibration_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.targets);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct VisualeyezCalibration_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ff_msgs::VisualeyezCalibration_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ff_msgs::VisualeyezCalibration_<ContainerAllocator>& v)
  {
    s << indent << "targets[]" << std::endl;
    for (size_t i = 0; i < v.targets.size(); ++i)
    {
      s << indent << "  targets[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::ff_msgs::VisualeyezDataArray_<ContainerAllocator> >::stream(s, indent + "    ", v.targets[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // FF_MSGS_MESSAGE_VISUALEYEZCALIBRATION_H