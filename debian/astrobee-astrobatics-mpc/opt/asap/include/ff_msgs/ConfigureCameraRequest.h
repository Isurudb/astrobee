// Generated by gencpp from file ff_msgs/ConfigureCameraRequest.msg
// DO NOT EDIT!


#ifndef FF_MSGS_MESSAGE_CONFIGURECAMERAREQUEST_H
#define FF_MSGS_MESSAGE_CONFIGURECAMERAREQUEST_H


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
struct ConfigureCameraRequest_
{
  typedef ConfigureCameraRequest_<ContainerAllocator> Type;

  ConfigureCameraRequest_()
    : mode(0)
    , rate(0.0)
    , width(0)
    , height(0)
    , bitrate(0.0)  {
    }
  ConfigureCameraRequest_(const ContainerAllocator& _alloc)
    : mode(0)
    , rate(0.0)
    , width(0)
    , height(0)
    , bitrate(0.0)  {
  (void)_alloc;
    }



   typedef uint8_t _mode_type;
  _mode_type mode;

   typedef float _rate_type;
  _rate_type rate;

   typedef uint32_t _width_type;
  _width_type width;

   typedef uint32_t _height_type;
  _height_type height;

   typedef float _bitrate_type;
  _bitrate_type bitrate;



  enum {
    BOTH = 0u,
    RECORDING = 1u,
    STREAMING = 2u,
  };


  typedef boost::shared_ptr< ::ff_msgs::ConfigureCameraRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ff_msgs::ConfigureCameraRequest_<ContainerAllocator> const> ConstPtr;

}; // struct ConfigureCameraRequest_

typedef ::ff_msgs::ConfigureCameraRequest_<std::allocator<void> > ConfigureCameraRequest;

typedef boost::shared_ptr< ::ff_msgs::ConfigureCameraRequest > ConfigureCameraRequestPtr;
typedef boost::shared_ptr< ::ff_msgs::ConfigureCameraRequest const> ConfigureCameraRequestConstPtr;

// constants requiring out of line definition

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ff_msgs::ConfigureCameraRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ff_msgs::ConfigureCameraRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::ff_msgs::ConfigureCameraRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ff_msgs::ConfigureCameraRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::ConfigureCameraRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::ConfigureCameraRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::ConfigureCameraRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::ConfigureCameraRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ff_msgs::ConfigureCameraRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "263cde84e0c4384e57b9ce048385281d";
  }

  static const char* value(const ::ff_msgs::ConfigureCameraRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x263cde84e0c4384eULL;
  static const uint64_t static_value2 = 0x57b9ce048385281dULL;
};

template<class ContainerAllocator>
struct DataType< ::ff_msgs::ConfigureCameraRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff_msgs/ConfigureCameraRequest";
  }

  static const char* value(const ::ff_msgs::ConfigureCameraRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ff_msgs::ConfigureCameraRequest_<ContainerAllocator> >
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
uint8 BOTH      = 0\n\
uint8 RECORDING = 1\n\
uint8 STREAMING = 2\n\
\n\
uint8 mode\n\
float32 rate\n\
uint32 width\n\
uint32 height\n\
float32 bitrate\n\
";
  }

  static const char* value(const ::ff_msgs::ConfigureCameraRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ff_msgs::ConfigureCameraRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.mode);
      stream.next(m.rate);
      stream.next(m.width);
      stream.next(m.height);
      stream.next(m.bitrate);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ConfigureCameraRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ff_msgs::ConfigureCameraRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ff_msgs::ConfigureCameraRequest_<ContainerAllocator>& v)
  {
    s << indent << "mode: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.mode);
    s << indent << "rate: ";
    Printer<float>::stream(s, indent + "  ", v.rate);
    s << indent << "width: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.width);
    s << indent << "height: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.height);
    s << indent << "bitrate: ";
    Printer<float>::stream(s, indent + "  ", v.bitrate);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FF_MSGS_MESSAGE_CONFIGURECAMERAREQUEST_H
