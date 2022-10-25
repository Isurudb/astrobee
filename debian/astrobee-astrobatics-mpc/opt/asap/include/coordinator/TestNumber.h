// Generated by gencpp from file coordinator/TestNumber.msg
// DO NOT EDIT!


#ifndef COORDINATOR_MESSAGE_TESTNUMBER_H
#define COORDINATOR_MESSAGE_TESTNUMBER_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace coordinator
{
template <class ContainerAllocator>
struct TestNumber_
{
  typedef TestNumber_<ContainerAllocator> Type;

  TestNumber_()
    : stamp()
    , test_number(0)
    , role()  {
    }
  TestNumber_(const ContainerAllocator& _alloc)
    : stamp()
    , test_number(0)
    , role(_alloc)  {
  (void)_alloc;
    }



   typedef ros::Time _stamp_type;
  _stamp_type stamp;

   typedef int32_t _test_number_type;
  _test_number_type test_number;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _role_type;
  _role_type role;





  typedef boost::shared_ptr< ::coordinator::TestNumber_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::coordinator::TestNumber_<ContainerAllocator> const> ConstPtr;

}; // struct TestNumber_

typedef ::coordinator::TestNumber_<std::allocator<void> > TestNumber;

typedef boost::shared_ptr< ::coordinator::TestNumber > TestNumberPtr;
typedef boost::shared_ptr< ::coordinator::TestNumber const> TestNumberConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::coordinator::TestNumber_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::coordinator::TestNumber_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace coordinator

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'coordinator': ['/home/isuru/Forked_astrobee/astrobee/src/asap/coordinator/msg'], 'std_msgs': ['/home/isuru/arm_cross/rootfs/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/home/isuru/arm_cross/rootfs/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::coordinator::TestNumber_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::coordinator::TestNumber_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::coordinator::TestNumber_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::coordinator::TestNumber_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::coordinator::TestNumber_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::coordinator::TestNumber_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::coordinator::TestNumber_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a546cf58ee360e93604091100205de8f";
  }

  static const char* value(const ::coordinator::TestNumber_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa546cf58ee360e93ULL;
  static const uint64_t static_value2 = 0x604091100205de8fULL;
};

template<class ContainerAllocator>
struct DataType< ::coordinator::TestNumber_<ContainerAllocator> >
{
  static const char* value()
  {
    return "coordinator/TestNumber";
  }

  static const char* value(const ::coordinator::TestNumber_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::coordinator::TestNumber_<ContainerAllocator> >
{
  static const char* value()
  {
    return "time stamp\n\
int32 test_number\n\
string role\n\
";
  }

  static const char* value(const ::coordinator::TestNumber_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::coordinator::TestNumber_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.stamp);
      stream.next(m.test_number);
      stream.next(m.role);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TestNumber_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::coordinator::TestNumber_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::coordinator::TestNumber_<ContainerAllocator>& v)
  {
    s << indent << "stamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.stamp);
    s << indent << "test_number: ";
    Printer<int32_t>::stream(s, indent + "  ", v.test_number);
    s << indent << "role: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.role);
  }
};

} // namespace message_operations
} // namespace ros

#endif // COORDINATOR_MESSAGE_TESTNUMBER_H
