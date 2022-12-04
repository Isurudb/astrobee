// Generated by gencpp from file ff_msgs/LocalizationResult.msg
// DO NOT EDIT!


#ifndef FF_MSGS_MESSAGE_LOCALIZATIONRESULT_H
#define FF_MSGS_MESSAGE_LOCALIZATIONRESULT_H


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
struct LocalizationResult_
{
  typedef LocalizationResult_<ContainerAllocator> Type;

  LocalizationResult_()
    : response(0)
    , fsm_result()  {
    }
  LocalizationResult_(const ContainerAllocator& _alloc)
    : response(0)
    , fsm_result(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _response_type;
  _response_type response;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _fsm_result_type;
  _fsm_result_type fsm_result;



  enum {
    PIPELINE_ALREADY_ACTIVE = 2,
    SUCCESS = 1,
    PREEMPTED = 0,
    CANCELLED = -1,
    INVALID_PIPELINE = -2,
    INVALID_COMMAND = -3,
    FILTER_NOT_IN_USE = -4,
    OPTICAL_FLOW_FAILED = -5,
    PIPELINE_TOGGLE_FAILED = -6,
    PIPELINE_USE_FAILED = -7,
    PIPELINE_UNSTABLE = -8,
    SET_INPUT_FAILED = -9,
    ESTIMATE_BIAS_FAILED = -10,
    RESET_FAILED = -11,
  };


  typedef boost::shared_ptr< ::ff_msgs::LocalizationResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ff_msgs::LocalizationResult_<ContainerAllocator> const> ConstPtr;

}; // struct LocalizationResult_

typedef ::ff_msgs::LocalizationResult_<std::allocator<void> > LocalizationResult;

typedef boost::shared_ptr< ::ff_msgs::LocalizationResult > LocalizationResultPtr;
typedef boost::shared_ptr< ::ff_msgs::LocalizationResult const> LocalizationResultConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ff_msgs::LocalizationResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ff_msgs::LocalizationResult_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::ff_msgs::LocalizationResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ff_msgs::LocalizationResult_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::LocalizationResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::LocalizationResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::LocalizationResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::LocalizationResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ff_msgs::LocalizationResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "309c1ead50fb170acfed9e9b67a66d27";
  }

  static const char* value(const ::ff_msgs::LocalizationResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x309c1ead50fb170aULL;
  static const uint64_t static_value2 = 0xcfed9e9b67a66d27ULL;
};

template<class ContainerAllocator>
struct DataType< ::ff_msgs::LocalizationResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff_msgs/LocalizationResult";
  }

  static const char* value(const ::ff_msgs::LocalizationResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ff_msgs::LocalizationResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
int32 response\n\
int32 PIPELINE_ALREADY_ACTIVE     =  2   # We are already on this mode\n\
int32 SUCCESS                     =  1   # The switch was successful\n\
int32 PREEMPTED                   =  0   # Preempted by another action goal\n\
int32 CANCELLED                   = -1   # We canceled our own request\n\
int32 INVALID_PIPELINE            = -2   # Not a valid pipeline in command\n\
int32 INVALID_COMMAND             = -3   # Not a valid command type\n\
int32 FILTER_NOT_IN_USE           = -4   # Reset/bias requires filter\n\
int32 OPTICAL_FLOW_FAILED         = -5   # Optical flow could not be toggled\n\
int32 PIPELINE_TOGGLE_FAILED      = -6   # Pipeline could not be toggled\n\
int32 PIPELINE_USE_FAILED         = -7   # Pipeline could not be used\n\
int32 PIPELINE_UNSTABLE           = -8   # Pipeline did not go stable\n\
int32 SET_INPUT_FAILED            = -9   # EKF could not be set to new mode\n\
int32 ESTIMATE_BIAS_FAILED        = -10  # Estimate bias service call failed\n\
int32 RESET_FAILED                = -11  # Reset service call failed\n\
\n\
# Human readable FSM result for debugging\n\
string fsm_result\n\
\n\
";
  }

  static const char* value(const ::ff_msgs::LocalizationResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ff_msgs::LocalizationResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.response);
      stream.next(m.fsm_result);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LocalizationResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ff_msgs::LocalizationResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ff_msgs::LocalizationResult_<ContainerAllocator>& v)
  {
    s << indent << "response: ";
    Printer<int32_t>::stream(s, indent + "  ", v.response);
    s << indent << "fsm_result: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.fsm_result);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FF_MSGS_MESSAGE_LOCALIZATIONRESULT_H