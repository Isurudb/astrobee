# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from ff_msgs/MotionFeedback.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import ff_msgs.msg
import geometry_msgs.msg
import genpy
import std_msgs.msg

class MotionFeedback(genpy.Message):
  _md5sum = "a70f7ffb0db5b74cf7c14491a515651c"
  _type = "ff_msgs/MotionFeedback"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

# The state of the teleop command
ff_msgs/MotionState state

# Control progress
ff_msgs/ControlFeedback progress

# Planner progress
float32 perc_complete
float32 secs_remaining


================================================================================
MSG: ff_msgs/MotionState
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.
#
# Locked topic that registers updates to the internal dock state

# Header with timestamp
std_msgs/Header header

# The state of the mobility subsystem
int8 state
int8 INITIALIZING        = 0
int8 IDLE                = 1
int8 STOPPED             = 2
int8 IDLING              = 3
int8 STOPPING            = 4
int8 PREPPING            = 5
int8 BOOTSTRAPPING       = 6
int8 PLANNING            = 7
int8 PREPARING           = 8
int8 CONTROLLING         = 9
int8 REPLANNING          = 10
int8 HALTING             = 11
int8 REPLAN_WAIT         = 12

# A human readble version of the (event) -> [state] transition
string fsm_event
string fsm_state

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: ff_msgs/ControlFeedback
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

uint32 index                                # Index being processed

ff_msgs/ControlState setpoint               # Current setpoint

float32 error_position                      # Position error
float32 error_attitude                      # Attitude error
float32 error_velocity                      # Velocity error
float32 error_omega                         # Omega error


================================================================================
MSG: ff_msgs/ControlState
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
# 
# All rights reserved.
# 
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.
#
# Full state vector containing Time, Pose, Vel, and Accel
# 
# when {time}
# flight_mode {string} - disctates, gains, tolerances, etc.
# pose {Point position, Quaternion orientation}
# twist {Vector3 linear, Vector3 angular}
# accel {Vector3 linear, Vector3 angular}

time when
geometry_msgs/Pose pose
geometry_msgs/Twist twist
geometry_msgs/Twist accel

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z"""
  __slots__ = ['state','progress','perc_complete','secs_remaining']
  _slot_types = ['ff_msgs/MotionState','ff_msgs/ControlFeedback','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       state,progress,perc_complete,secs_remaining

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MotionFeedback, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.state is None:
        self.state = ff_msgs.msg.MotionState()
      if self.progress is None:
        self.progress = ff_msgs.msg.ControlFeedback()
      if self.perc_complete is None:
        self.perc_complete = 0.
      if self.secs_remaining is None:
        self.secs_remaining = 0.
    else:
      self.state = ff_msgs.msg.MotionState()
      self.progress = ff_msgs.msg.ControlFeedback()
      self.perc_complete = 0.
      self.secs_remaining = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.state.header.seq, _x.state.header.stamp.secs, _x.state.header.stamp.nsecs))
      _x = self.state.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_get_struct_b().pack(self.state.state))
      _x = self.state.fsm_event
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.state.fsm_state
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_3I19d6f().pack(_x.progress.index, _x.progress.setpoint.when.secs, _x.progress.setpoint.when.nsecs, _x.progress.setpoint.pose.position.x, _x.progress.setpoint.pose.position.y, _x.progress.setpoint.pose.position.z, _x.progress.setpoint.pose.orientation.x, _x.progress.setpoint.pose.orientation.y, _x.progress.setpoint.pose.orientation.z, _x.progress.setpoint.pose.orientation.w, _x.progress.setpoint.twist.linear.x, _x.progress.setpoint.twist.linear.y, _x.progress.setpoint.twist.linear.z, _x.progress.setpoint.twist.angular.x, _x.progress.setpoint.twist.angular.y, _x.progress.setpoint.twist.angular.z, _x.progress.setpoint.accel.linear.x, _x.progress.setpoint.accel.linear.y, _x.progress.setpoint.accel.linear.z, _x.progress.setpoint.accel.angular.x, _x.progress.setpoint.accel.angular.y, _x.progress.setpoint.accel.angular.z, _x.progress.error_position, _x.progress.error_attitude, _x.progress.error_velocity, _x.progress.error_omega, _x.perc_complete, _x.secs_remaining))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.state is None:
        self.state = ff_msgs.msg.MotionState()
      if self.progress is None:
        self.progress = ff_msgs.msg.ControlFeedback()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.state.header.seq, _x.state.header.stamp.secs, _x.state.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.state.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.state.header.frame_id = str[start:end]
      start = end
      end += 1
      (self.state.state,) = _get_struct_b().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.state.fsm_event = str[start:end].decode('utf-8')
      else:
        self.state.fsm_event = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.state.fsm_state = str[start:end].decode('utf-8')
      else:
        self.state.fsm_state = str[start:end]
      _x = self
      start = end
      end += 188
      (_x.progress.index, _x.progress.setpoint.when.secs, _x.progress.setpoint.when.nsecs, _x.progress.setpoint.pose.position.x, _x.progress.setpoint.pose.position.y, _x.progress.setpoint.pose.position.z, _x.progress.setpoint.pose.orientation.x, _x.progress.setpoint.pose.orientation.y, _x.progress.setpoint.pose.orientation.z, _x.progress.setpoint.pose.orientation.w, _x.progress.setpoint.twist.linear.x, _x.progress.setpoint.twist.linear.y, _x.progress.setpoint.twist.linear.z, _x.progress.setpoint.twist.angular.x, _x.progress.setpoint.twist.angular.y, _x.progress.setpoint.twist.angular.z, _x.progress.setpoint.accel.linear.x, _x.progress.setpoint.accel.linear.y, _x.progress.setpoint.accel.linear.z, _x.progress.setpoint.accel.angular.x, _x.progress.setpoint.accel.angular.y, _x.progress.setpoint.accel.angular.z, _x.progress.error_position, _x.progress.error_attitude, _x.progress.error_velocity, _x.progress.error_omega, _x.perc_complete, _x.secs_remaining,) = _get_struct_3I19d6f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.state.header.seq, _x.state.header.stamp.secs, _x.state.header.stamp.nsecs))
      _x = self.state.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_get_struct_b().pack(self.state.state))
      _x = self.state.fsm_event
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.state.fsm_state
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_3I19d6f().pack(_x.progress.index, _x.progress.setpoint.when.secs, _x.progress.setpoint.when.nsecs, _x.progress.setpoint.pose.position.x, _x.progress.setpoint.pose.position.y, _x.progress.setpoint.pose.position.z, _x.progress.setpoint.pose.orientation.x, _x.progress.setpoint.pose.orientation.y, _x.progress.setpoint.pose.orientation.z, _x.progress.setpoint.pose.orientation.w, _x.progress.setpoint.twist.linear.x, _x.progress.setpoint.twist.linear.y, _x.progress.setpoint.twist.linear.z, _x.progress.setpoint.twist.angular.x, _x.progress.setpoint.twist.angular.y, _x.progress.setpoint.twist.angular.z, _x.progress.setpoint.accel.linear.x, _x.progress.setpoint.accel.linear.y, _x.progress.setpoint.accel.linear.z, _x.progress.setpoint.accel.angular.x, _x.progress.setpoint.accel.angular.y, _x.progress.setpoint.accel.angular.z, _x.progress.error_position, _x.progress.error_attitude, _x.progress.error_velocity, _x.progress.error_omega, _x.perc_complete, _x.secs_remaining))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.state is None:
        self.state = ff_msgs.msg.MotionState()
      if self.progress is None:
        self.progress = ff_msgs.msg.ControlFeedback()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.state.header.seq, _x.state.header.stamp.secs, _x.state.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.state.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.state.header.frame_id = str[start:end]
      start = end
      end += 1
      (self.state.state,) = _get_struct_b().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.state.fsm_event = str[start:end].decode('utf-8')
      else:
        self.state.fsm_event = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.state.fsm_state = str[start:end].decode('utf-8')
      else:
        self.state.fsm_state = str[start:end]
      _x = self
      start = end
      end += 188
      (_x.progress.index, _x.progress.setpoint.when.secs, _x.progress.setpoint.when.nsecs, _x.progress.setpoint.pose.position.x, _x.progress.setpoint.pose.position.y, _x.progress.setpoint.pose.position.z, _x.progress.setpoint.pose.orientation.x, _x.progress.setpoint.pose.orientation.y, _x.progress.setpoint.pose.orientation.z, _x.progress.setpoint.pose.orientation.w, _x.progress.setpoint.twist.linear.x, _x.progress.setpoint.twist.linear.y, _x.progress.setpoint.twist.linear.z, _x.progress.setpoint.twist.angular.x, _x.progress.setpoint.twist.angular.y, _x.progress.setpoint.twist.angular.z, _x.progress.setpoint.accel.linear.x, _x.progress.setpoint.accel.linear.y, _x.progress.setpoint.accel.linear.z, _x.progress.setpoint.accel.angular.x, _x.progress.setpoint.accel.angular.y, _x.progress.setpoint.accel.angular.z, _x.progress.error_position, _x.progress.error_attitude, _x.progress.error_velocity, _x.progress.error_omega, _x.perc_complete, _x.secs_remaining,) = _get_struct_3I19d6f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I19d6f = None
def _get_struct_3I19d6f():
    global _struct_3I19d6f
    if _struct_3I19d6f is None:
        _struct_3I19d6f = struct.Struct("<3I19d6f")
    return _struct_3I19d6f
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_b = None
def _get_struct_b():
    global _struct_b
    if _struct_b is None:
        _struct_b = struct.Struct("<b")
    return _struct_b