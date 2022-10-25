// Generated by gencpp from file ff_msgs/SetStateRequest.msg
// DO NOT EDIT!


#ifndef FF_MSGS_MESSAGE_SETSTATEREQUEST_H
#define FF_MSGS_MESSAGE_SETSTATEREQUEST_H


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
struct SetStateRequest_
{
  typedef SetStateRequest_<ContainerAllocator> Type;

  SetStateRequest_()
    : state(0)  {
    }
  SetStateRequest_(const ContainerAllocator& _alloc)
    : state(0)  {
  (void)_alloc;
    }



   typedef int32_t _state_type;
  _state_type state;





  typedef boost::shared_ptr< ::ff_msgs::SetStateRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ff_msgs::SetStateRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetStateRequest_

typedef ::ff_msgs::SetStateRequest_<std::allocator<void> > SetStateRequest;

typedef boost::shared_ptr< ::ff_msgs::SetStateRequest > SetStateRequestPtr;
typedef boost::shared_ptr< ::ff_msgs::SetStateRequest const> SetStateRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ff_msgs::SetStateRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ff_msgs::SetStateRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::ff_msgs::SetStateRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ff_msgs::SetStateRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::SetStateRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::SetStateRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::SetStateRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::SetStateRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ff_msgs::SetStateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7a2f37ef2ba405f0c7a15cc72663d6f0";
  }

  static const char* value(const ::ff_msgs::SetStateRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7a2f37ef2ba405f0ULL;
  static const uint64_t static_value2 = 0xc7a15cc72663d6f0ULL;
};

template<class ContainerAllocator>
struct DataType< ::ff_msgs::SetStateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff_msgs/SetStateRequest";
  }

  static const char* value(const ::ff_msgs::SetStateRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ff_msgs::SetStateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
int32 state\n\
";
  }

  static const char* value(const ::ff_msgs::SetStateRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ff_msgs::SetStateRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetStateRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ff_msgs::SetStateRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ff_msgs::SetStateRequest_<ContainerAllocator>& v)
  {
    s << indent << "state: ";
    Printer<int32_t>::stream(s, indent + "  ", v.state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FF_MSGS_MESSAGE_SETSTATEREQUEST_H
