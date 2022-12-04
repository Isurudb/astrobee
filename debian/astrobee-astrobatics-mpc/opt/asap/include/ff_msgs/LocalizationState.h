// Generated by gencpp from file ff_msgs/LocalizationState.msg
// DO NOT EDIT!


#ifndef FF_MSGS_MESSAGE_LOCALIZATIONSTATE_H
#define FF_MSGS_MESSAGE_LOCALIZATIONSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <ff_msgs/LocalizationPipeline.h>

namespace ff_msgs
{
template <class ContainerAllocator>
struct LocalizationState_
{
  typedef LocalizationState_<ContainerAllocator> Type;

  LocalizationState_()
    : header()
    , state(0)
    , fsm_event()
    , fsm_state()
    , pipeline()  {
    }
  LocalizationState_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , state(0)
    , fsm_event(_alloc)
    , fsm_state(_alloc)
    , pipeline(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int32_t _state_type;
  _state_type state;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _fsm_event_type;
  _fsm_event_type fsm_event;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _fsm_state_type;
  _fsm_state_type fsm_state;

   typedef  ::ff_msgs::LocalizationPipeline_<ContainerAllocator>  _pipeline_type;
  _pipeline_type pipeline;



  enum {
    INITIALIZING = 0,
    DISABLED = 1,
    LOCALIZING = 2,
    SWITCH_WAITING_FOR_PIPELINE = 3,
    SWITCH_WAITING_FOR_FILTER = 4,
    BIAS_WAITING_FOR_FILTER = 5,
    RESET_WAITING_FOR_FILTER = 6,
    UNSTABLE = 7,
  };


  typedef boost::shared_ptr< ::ff_msgs::LocalizationState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ff_msgs::LocalizationState_<ContainerAllocator> const> ConstPtr;

}; // struct LocalizationState_

typedef ::ff_msgs::LocalizationState_<std::allocator<void> > LocalizationState;

typedef boost::shared_ptr< ::ff_msgs::LocalizationState > LocalizationStatePtr;
typedef boost::shared_ptr< ::ff_msgs::LocalizationState const> LocalizationStateConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ff_msgs::LocalizationState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ff_msgs::LocalizationState_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::ff_msgs::LocalizationState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ff_msgs::LocalizationState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::LocalizationState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::LocalizationState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::LocalizationState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::LocalizationState_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ff_msgs::LocalizationState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4fe8f08dfd156a0a44226bd8862089d4";
  }

  static const char* value(const ::ff_msgs::LocalizationState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4fe8f08dfd156a0aULL;
  static const uint64_t static_value2 = 0x44226bd8862089d4ULL;
};

template<class ContainerAllocator>
struct DataType< ::ff_msgs::LocalizationState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff_msgs/LocalizationState";
  }

  static const char* value(const ::ff_msgs::LocalizationState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ff_msgs::LocalizationState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Copyright (c) 2017, United States Government, as represented by the\n\
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
# The state of the localization system\n\
\n\
# Header with timestamp\n\
std_msgs/Header header\n\
\n\
# Tee current state\n\
int32 state                                 # Current state\n\
int32 INITIALIZING                    = 0   # Waiting on dependencies\n\
int32 DISABLED                        = 1   # Localization disabled\n\
int32 LOCALIZING                      = 2   # Localization enabled\n\
int32 SWITCH_WAITING_FOR_PIPELINE     = 3   # Waiting for pipeline to stabilize\n\
int32 SWITCH_WAITING_FOR_FILTER       = 4   # Waiting for filter to stabilize\n\
int32 BIAS_WAITING_FOR_FILTER         = 5   # Waiting for bias estimation\n\
int32 RESET_WAITING_FOR_FILTER        = 6   # Waiting for EKF stability\n\
int32 UNSTABLE                        = 7   # Fallback pipeline unstable\n\
\n\
# A human readable version of the (event) -> [state] transition\n\
string fsm_event\n\
string fsm_state\n\
\n\
# The current localization pipeline being used\n\
ff_msgs/LocalizationPipeline pipeline\n\
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

  static const char* value(const ::ff_msgs::LocalizationState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ff_msgs::LocalizationState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.state);
      stream.next(m.fsm_event);
      stream.next(m.fsm_state);
      stream.next(m.pipeline);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LocalizationState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ff_msgs::LocalizationState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ff_msgs::LocalizationState_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "state: ";
    Printer<int32_t>::stream(s, indent + "  ", v.state);
    s << indent << "fsm_event: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.fsm_event);
    s << indent << "fsm_state: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.fsm_state);
    s << indent << "pipeline: ";
    s << std::endl;
    Printer< ::ff_msgs::LocalizationPipeline_<ContainerAllocator> >::stream(s, indent + "  ", v.pipeline);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FF_MSGS_MESSAGE_LOCALIZATIONSTATE_H