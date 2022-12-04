// Generated by gencpp from file ff_msgs/PlanResult.msg
// DO NOT EDIT!


#ifndef FF_MSGS_MESSAGE_PLANRESULT_H
#define FF_MSGS_MESSAGE_PLANRESULT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <ff_msgs/ControlState.h>

namespace ff_msgs
{
template <class ContainerAllocator>
struct PlanResult_
{
  typedef PlanResult_<ContainerAllocator> Type;

  PlanResult_()
    : response(0)
    , segment()  {
    }
  PlanResult_(const ContainerAllocator& _alloc)
    : response(0)
    , segment(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _response_type;
  _response_type response;

   typedef std::vector< ::ff_msgs::ControlState_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::ff_msgs::ControlState_<ContainerAllocator> >::other >  _segment_type;
  _segment_type segment;



  enum {
    ALREADY_THERE = 3,
    CANCELLED = 2,
    SUCCESS = 1,
    PREEMPTED = 0,
    NOT_ENOUGH_STATES = -1,
    OBSTACLES_NOT_SUPPORTED = -2,
    BAD_STATE_TRANSITION = -3,
    CANNOT_LOAD_FLIGHT_DATA = -4,
    CANNOT_LOAD_GENERAL_CONFIG = -5,
    NO_PATH_EXISTS = -6,
    PROBLEM_CONNECTING_TO_SERVICES = -7,
    BAD_ARGUMENTS = -8,
  };


  typedef boost::shared_ptr< ::ff_msgs::PlanResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ff_msgs::PlanResult_<ContainerAllocator> const> ConstPtr;

}; // struct PlanResult_

typedef ::ff_msgs::PlanResult_<std::allocator<void> > PlanResult;

typedef boost::shared_ptr< ::ff_msgs::PlanResult > PlanResultPtr;
typedef boost::shared_ptr< ::ff_msgs::PlanResult const> PlanResultConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ff_msgs::PlanResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ff_msgs::PlanResult_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::ff_msgs::PlanResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ff_msgs::PlanResult_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::PlanResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::PlanResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::PlanResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::PlanResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ff_msgs::PlanResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "910a7e28b72d9276acb77d29399efa00";
  }

  static const char* value(const ::ff_msgs::PlanResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x910a7e28b72d9276ULL;
  static const uint64_t static_value2 = 0xacb77d29399efa00ULL;
};

template<class ContainerAllocator>
struct DataType< ::ff_msgs::PlanResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff_msgs/PlanResult";
  }

  static const char* value(const ::ff_msgs::PlanResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ff_msgs::PlanResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
int32 response                                # Response\n\
int32 ALREADY_THERE                    =  3\n\
int32 CANCELLED                        =  2\n\
int32 SUCCESS                          =  1\n\
int32 PREEMPTED                        =  0\n\
int32 NOT_ENOUGH_STATES                = -1\n\
int32 OBSTACLES_NOT_SUPPORTED          = -2\n\
int32 BAD_STATE_TRANSITION             = -3\n\
int32 CANNOT_LOAD_FLIGHT_DATA          = -4\n\
int32 CANNOT_LOAD_GENERAL_CONFIG       = -5\n\
int32 NO_PATH_EXISTS                   = -6\n\
int32 PROBLEM_CONNECTING_TO_SERVICES   = -7\n\
int32 BAD_ARGUMENTS                    = -8\n\
\n\
ff_msgs/ControlState[] segment                # Output segment\n\
\n\
\n\
================================================================================\n\
MSG: ff_msgs/ControlState\n\
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
# Full state vector containing Time, Pose, Vel, and Accel\n\
# \n\
# when {time}\n\
# flight_mode {string} - disctates, gains, tolerances, etc.\n\
# pose {Point position, Quaternion orientation}\n\
# twist {Vector3 linear, Vector3 angular}\n\
# accel {Vector3 linear, Vector3 angular}\n\
\n\
time when\n\
geometry_msgs/Pose pose\n\
geometry_msgs/Twist twist\n\
geometry_msgs/Twist accel\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of position and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Twist\n\
# This expresses velocity in free space broken into its linear and angular parts.\n\
Vector3  linear\n\
Vector3  angular\n\
\n\
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

  static const char* value(const ::ff_msgs::PlanResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ff_msgs::PlanResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.response);
      stream.next(m.segment);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PlanResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ff_msgs::PlanResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ff_msgs::PlanResult_<ContainerAllocator>& v)
  {
    s << indent << "response: ";
    Printer<int32_t>::stream(s, indent + "  ", v.response);
    s << indent << "segment[]" << std::endl;
    for (size_t i = 0; i < v.segment.size(); ++i)
    {
      s << indent << "  segment[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::ff_msgs::ControlState_<ContainerAllocator> >::stream(s, indent + "    ", v.segment[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // FF_MSGS_MESSAGE_PLANRESULT_H