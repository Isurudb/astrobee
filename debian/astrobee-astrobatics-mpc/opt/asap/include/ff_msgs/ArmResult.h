// Generated by gencpp from file ff_msgs/ArmResult.msg
// DO NOT EDIT!


#ifndef FF_MSGS_MESSAGE_ARMRESULT_H
#define FF_MSGS_MESSAGE_ARMRESULT_H


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
struct ArmResult_
{
  typedef ArmResult_<ContainerAllocator> Type;

  ArmResult_()
    : response(0)
    , fsm_result()  {
    }
  ArmResult_(const ContainerAllocator& _alloc)
    : response(0)
    , fsm_result(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _response_type;
  _response_type response;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _fsm_result_type;
  _fsm_result_type fsm_result;



  enum {
    SUCCESS = 1,
    PREEMPTED = 0,
    INVALID_COMMAND = -1,
    BAD_TILT_VALUE = -2,
    BAD_PAN_VALUE = -3,
    BAD_GRIPPER_VALUE = -4,
    NOT_ALLOWED = -5,
    TILT_FAILED = -6,
    PAN_FAILED = -7,
    GRIPPER_FAILED = -8,
    COMMUNICATION_ERROR = -9,
    COLLISION_AVOIDED = -10,
    ENABLE_FAILED = -11,
    DISABLE_FAILED = -12,
    CALIBRATE_FAILED = -13,
    NO_GOAL = -14,
  };


  typedef boost::shared_ptr< ::ff_msgs::ArmResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ff_msgs::ArmResult_<ContainerAllocator> const> ConstPtr;

}; // struct ArmResult_

typedef ::ff_msgs::ArmResult_<std::allocator<void> > ArmResult;

typedef boost::shared_ptr< ::ff_msgs::ArmResult > ArmResultPtr;
typedef boost::shared_ptr< ::ff_msgs::ArmResult const> ArmResultConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ff_msgs::ArmResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ff_msgs::ArmResult_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::ff_msgs::ArmResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ff_msgs::ArmResult_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::ArmResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::ArmResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::ArmResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::ArmResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ff_msgs::ArmResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5c229b93f1064b9f1f7e8f3320eff359";
  }

  static const char* value(const ::ff_msgs::ArmResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5c229b93f1064b9fULL;
  static const uint64_t static_value2 = 0x1f7e8f3320eff359ULL;
};

template<class ContainerAllocator>
struct DataType< ::ff_msgs::ArmResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff_msgs/ArmResult";
  }

  static const char* value(const ::ff_msgs::ArmResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ff_msgs::ArmResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
# Machine-readable reseult code\n\
int32 response\n\
int32 SUCCESS             =  1                # Successfully completed\n\
int32 PREEMPTED           =  0                # Action was preempted\n\
int32 INVALID_COMMAND     = -1                # Invalid command\n\
int32 BAD_TILT_VALUE      = -2                # Invalid value for tilt\n\
int32 BAD_PAN_VALUE       = -3                # Invalid value for pan\n\
int32 BAD_GRIPPER_VALUE   = -4                # Invalid value for gripper\n\
int32 NOT_ALLOWED         = -5                # Not allowed\n\
int32 TILT_FAILED         = -6                # Tilt command failed\n\
int32 PAN_FAILED          = -7                # Pan command failed\n\
int32 GRIPPER_FAILED      = -8                # Gripper command failed\n\
int32 COMMUNICATION_ERROR = -9                # Cannot communicate with arm\n\
int32 COLLISION_AVOIDED   = -10               # No panning when tilt < 90\n\
int32 ENABLE_FAILED       = -11               # Cannot enable the servos\n\
int32 DISABLE_FAILED      = -12               # Cannot disable the servos\n\
int32 CALIBRATE_FAILED    = -13               # Cannot calibrate the gripper\n\
int32 NO_GOAL             = -14               # Unknown call to calibration\n\
\n\
# Human readable FSM result for debugging\n\
string fsm_result\n\
\n\
";
  }

  static const char* value(const ::ff_msgs::ArmResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ff_msgs::ArmResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.response);
      stream.next(m.fsm_result);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ArmResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ff_msgs::ArmResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ff_msgs::ArmResult_<ContainerAllocator>& v)
  {
    s << indent << "response: ";
    Printer<int32_t>::stream(s, indent + "  ", v.response);
    s << indent << "fsm_result: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.fsm_result);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FF_MSGS_MESSAGE_ARMRESULT_H