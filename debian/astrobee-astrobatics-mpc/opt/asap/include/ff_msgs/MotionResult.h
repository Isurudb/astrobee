// Generated by gencpp from file ff_msgs/MotionResult.msg
// DO NOT EDIT!


#ifndef FF_MSGS_MESSAGE_MOTIONRESULT_H
#define FF_MSGS_MESSAGE_MOTIONRESULT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <ff_msgs/FlightMode.h>
#include <ff_msgs/ControlState.h>

namespace ff_msgs
{
template <class ContainerAllocator>
struct MotionResult_
{
  typedef MotionResult_<ContainerAllocator> Type;

  MotionResult_()
    : response(0)
    , fsm_result()
    , flight_mode()
    , segment()  {
    }
  MotionResult_(const ContainerAllocator& _alloc)
    : response(0)
    , fsm_result(_alloc)
    , flight_mode(_alloc)
    , segment(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _response_type;
  _response_type response;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _fsm_result_type;
  _fsm_result_type fsm_result;

   typedef  ::ff_msgs::FlightMode_<ContainerAllocator>  _flight_mode_type;
  _flight_mode_type flight_mode;

   typedef std::vector< ::ff_msgs::ControlState_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::ff_msgs::ControlState_<ContainerAllocator> >::other >  _segment_type;
  _segment_type segment;



  enum {
    ALREADY_THERE = 2,
    SUCCESS = 1,
    PREEMPTED = 0,
    PLAN_FAILED = -1,
    VALIDATE_FAILED = -2,
    PMC_FAILED = -3,
    CONTROL_FAILED = -4,
    OBSTACLE_DETECTED = -5,
    REPLAN_NOT_ENOUGH_TIME = -6,
    REPLAN_FAILED = -7,
    REVALIDATE_FAILED = -8,
    NOT_IN_WAITING_MODE = -9,
    INVALID_FLIGHT_MODE = -10,
    UNEXPECTED_EMPTY_SEGMENT = -11,
    COULD_NOT_RESAMPLE = -12,
    UNEXPECTED_EMPTY_STATES = -13,
    INVALID_COMMAND = -14,
    CANNOT_QUERY_ROBOT_POSE = -15,
    NOT_ON_FIRST_POSE = -16,
    BAD_DESIRED_VELOCITY = -17,
    BAD_DESIRED_ACCELERATION = -18,
    BAD_DESIRED_OMEGA = -19,
    BAD_DESIRED_ALPHA = -20,
    BAD_DESIRED_RATE = -21,
    TOLERANCE_VIOLATION_POSITION_ENDPOINT = -22,
    TOLERANCE_VIOLATION_POSITION = -23,
    TOLERANCE_VIOLATION_ATTITUDE = -24,
    TOLERANCE_VIOLATION_VELOCITY = -25,
    TOLERANCE_VIOLATION_OMEGA = -26,
    VIOLATES_RESAMPLING = -27,
    VIOLATES_KEEP_OUT = -28,
    VIOLATES_KEEP_IN = -29,
    VIOLATES_MINIMUM_FREQUENCY = -30,
    VIOLATES_STATIONARY_ENDPOINT = -31,
    VIOLATES_FIRST_IN_PAST = -32,
    VIOLATES_MINIMUM_SETPOINTS = -33,
    VIOLATES_HARD_LIMIT_VEL = -34,
    VIOLATES_HARD_LIMIT_ACCEL = -35,
    VIOLATES_HARD_LIMIT_OMEGA = -36,
    VIOLATES_HARD_LIMIT_ALPHA = -37,
    CANCELLED = -38,
    INVALID_REFERENCE_FRAME = -39,
  };


  typedef boost::shared_ptr< ::ff_msgs::MotionResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ff_msgs::MotionResult_<ContainerAllocator> const> ConstPtr;

}; // struct MotionResult_

typedef ::ff_msgs::MotionResult_<std::allocator<void> > MotionResult;

typedef boost::shared_ptr< ::ff_msgs::MotionResult > MotionResultPtr;
typedef boost::shared_ptr< ::ff_msgs::MotionResult const> MotionResultConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ff_msgs::MotionResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ff_msgs::MotionResult_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::ff_msgs::MotionResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ff_msgs::MotionResult_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::MotionResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ff_msgs::MotionResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::MotionResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ff_msgs::MotionResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ff_msgs::MotionResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d9085ddc2b12a9c56a85552d7ec1a05e";
  }

  static const char* value(const ::ff_msgs::MotionResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd9085ddc2b12a9c5ULL;
  static const uint64_t static_value2 = 0x6a85552d7ec1a05eULL;
};

template<class ContainerAllocator>
struct DataType< ::ff_msgs::MotionResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff_msgs/MotionResult";
  }

  static const char* value(const ::ff_msgs::MotionResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ff_msgs::MotionResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
# Motion result\n\
int32 response                            # Motion action response\n\
int32 ALREADY_THERE                         =   2  # MOVE: We are already at the location\n\
int32 SUCCESS                               =   1  # ALL: Motion succeeded\n\
int32 PREEMPTED                             =   0  # ALL: Motion preempted by thirdparty\n\
int32 PLAN_FAILED                           =  -1  # MOVE/EXEC: Plan/bootstrap failed\n\
int32 VALIDATE_FAILED                       =  -2  # MOVE/EXEC: No comms with mapper\n\
int32 PMC_FAILED                            =  -3  # MOVE/EXEC: PMC failed\n\
int32 CONTROL_FAILED                        =  -4  # ALL: Control failed\n\
int32 OBSTACLE_DETECTED                     =  -5  # ALL: Obstacle / replan disabled\n\
int32 REPLAN_NOT_ENOUGH_TIME                =  -6  # MOVE/EXEC: Not enough time to replan\n\
int32 REPLAN_FAILED                         =  -7  # MOVE/EXEC: Replanning failed\n\
int32 REVALIDATE_FAILED                     =  -8  # MOVE/EXEC: Revalidating failed\n\
int32 NOT_IN_WAITING_MODE                   =  -9  # ALL: Internal failure\n\
int32 INVALID_FLIGHT_MODE                   =  -10 # ALL: No flight mode specified\n\
int32 UNEXPECTED_EMPTY_SEGMENT              =  -11 # EXEC: Segment empty\n\
int32 COULD_NOT_RESAMPLE                    =  -12 # EXEC: Could not resample segment\n\
int32 UNEXPECTED_EMPTY_STATES               =  -13 # MOVE: State vector empty\n\
int32 INVALID_COMMAND                       =  -14 # Command rejected\n\
int32 CANNOT_QUERY_ROBOT_POSE               =  -15 # TF2 failed to find the current pose\n\
int32 NOT_ON_FIRST_POSE                     =  -16 # EXEC: Not on first pose of exec\n\
int32 BAD_DESIRED_VELOCITY                  =  -17 # Requested vel too high\n\
int32 BAD_DESIRED_ACCELERATION              =  -18 # Requested accel too high\n\
int32 BAD_DESIRED_OMEGA                     =  -19 # Requested omega too high\n\
int32 BAD_DESIRED_ALPHA                     =  -20 # Requested alpha too high\n\
int32 BAD_DESIRED_RATE                      =  -21 # Requested rate too low\n\
int32 TOLERANCE_VIOLATION_POSITION_ENDPOINT =  -22 # Position tolerance violated\n\
int32 TOLERANCE_VIOLATION_POSITION          =  -23 # Position tolerance violated\n\
int32 TOLERANCE_VIOLATION_ATTITUDE          =  -24 # Attitude tolerance violated\n\
int32 TOLERANCE_VIOLATION_VELOCITY          =  -25 # Velocity tolerance violated\n\
int32 TOLERANCE_VIOLATION_OMEGA             =  -26 # Omega tolerance violated\n\
int32 VIOLATES_RESAMPLING                   =  -27 # Validation: could not resample@10Hz\n\
int32 VIOLATES_KEEP_OUT                     =  -28 # Validation: Keep out violation\n\
int32 VIOLATES_KEEP_IN                      =  -29 # Validation: Keep in violation\n\
int32 VIOLATES_MINIMUM_FREQUENCY            =  -30 # Validation: Sample frequency too low\n\
int32 VIOLATES_STATIONARY_ENDPOINT          =  -31 # Validation: Last setpoint not static\n\
int32 VIOLATES_FIRST_IN_PAST                =  -32 # Validation: First timestamp in past\n\
int32 VIOLATES_MINIMUM_SETPOINTS            =  -33 # Validation: Not enough setpoints\n\
int32 VIOLATES_HARD_LIMIT_VEL               =  -34 # Validation: Velocity too high\n\
int32 VIOLATES_HARD_LIMIT_ACCEL             =  -35 # Validation: Acceleration too high\n\
int32 VIOLATES_HARD_LIMIT_OMEGA             =  -36 # Validation: Omega too high\n\
int32 VIOLATES_HARD_LIMIT_ALPHA             =  -37 # Validation: Alpha too high\n\
int32 CANCELLED                             =  -38 # ALL: Motion cancelled by callee\n\
int32 INVALID_REFERENCE_FRAME               =  -39 # ALL: Unknown reference frame\n\
\n\
# Human readable FSM result for debugging\n\
string fsm_result\n\
\n\
# The flight mode parameters used\n\
ff_msgs/FlightMode flight_mode\n\
\n\
# The final segment that was flown\n\
ff_msgs/ControlState[] segment\n\
\n\
\n\
================================================================================\n\
MSG: ff_msgs/FlightMode\n\
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
# This message captures all information in a flight mode\n\
\n\
Header header                     # Metadata\n\
\n\
string name                       # Name of the flight mode\n\
\n\
bool control_enabled              # Is control enabled?\n\
\n\
# Tolerances (all in SI units)\n\
float32 tolerance_pos_endpoint    # Endpoint position tolerance in m\n\
float32 tolerance_pos             # Position tolerance in m\n\
float32 tolerance_vel             # Velocity tolerance in m/s\n\
float32 tolerance_att             # Attitude tolerance in rads\n\
float32 tolerance_omega           # Angular acceleration tolerance in rad/s\n\
float32 tolerance_time            # Acceptable lag betwee TX and RX of control\n\
\n\
# Controller gains\n\
geometry_msgs/Vector3 att_kp      # Positional proportional constant\n\
geometry_msgs/Vector3 att_ki      # Positional integrative constant\n\
geometry_msgs/Vector3 omega_kd    # Attidue derivative constant\n\
geometry_msgs/Vector3 pos_kp      # Positional proportional contant\n\
geometry_msgs/Vector3 pos_ki      # Positional integrative constant\n\
geometry_msgs/Vector3 vel_kd      # Positional derivative constant\n\
\n\
# Hard limit on planning\n\
float32 hard_limit_vel            # Position tolerance in m/s\n\
float32 hard_limit_accel          # Position tolerance in m/s^2\n\
float32 hard_limit_omega          # Position tolerance in rads/s\n\
float32 hard_limit_alpha          # Position tolerance in rads/s^2\n\
\n\
# Impeller speed\n\
uint8 speed                       # Current speed gain\n\
uint8 SPEED_MIN        = 0        # Min acceptable gain\n\
uint8 SPEED_OFF        = 0        # Blowers off\n\
uint8 SPEED_QUIET      = 1        # Quiet mode\n\
uint8 SPEED_NOMINAL    = 2        # Nomainal mode\n\
uint8 SPEED_AGGRESSIVE = 3        # Aggressive mode\n\
uint8 SPEED_MAX        = 3        # Max acceptable gain\n\
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
";
  }

  static const char* value(const ::ff_msgs::MotionResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ff_msgs::MotionResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.response);
      stream.next(m.fsm_result);
      stream.next(m.flight_mode);
      stream.next(m.segment);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MotionResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ff_msgs::MotionResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ff_msgs::MotionResult_<ContainerAllocator>& v)
  {
    s << indent << "response: ";
    Printer<int32_t>::stream(s, indent + "  ", v.response);
    s << indent << "fsm_result: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.fsm_result);
    s << indent << "flight_mode: ";
    s << std::endl;
    Printer< ::ff_msgs::FlightMode_<ContainerAllocator> >::stream(s, indent + "  ", v.flight_mode);
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

#endif // FF_MSGS_MESSAGE_MOTIONRESULT_H
