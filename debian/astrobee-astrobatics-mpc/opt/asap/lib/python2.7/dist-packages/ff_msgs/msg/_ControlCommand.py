# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from ff_msgs/ControlCommand.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import ff_msgs.msg
import geometry_msgs.msg
import genpy
import std_msgs.msg

class ControlCommand(genpy.Message):
  _md5sum = "d82f6c881b46d5890f70c1dd8fdcfd1a"
  _type = "ff_msgs/ControlCommand"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """# Copyright (c) 2017, United States Government, as represented by the
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
# two copies of a ControlState message, plus a header
# mode: the current mode we are in. only two states defined so far
# current: the current ControlState trajectory we should aim for
# next: the next ControlState trajectory, just in case

std_msgs/Header header
uint8 mode
uint8 MODE_IDLE = 0
uint8 MODE_STOP = 1
uint8 MODE_NOMINAL = 2
ff_msgs/ControlState current
ff_msgs/ControlState next

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
  # Pseudo-constants
  MODE_IDLE = 0
  MODE_STOP = 1
  MODE_NOMINAL = 2

  __slots__ = ['header','mode','current','next']
  _slot_types = ['std_msgs/Header','uint8','ff_msgs/ControlState','ff_msgs/ControlState']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,mode,current,next

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ControlCommand, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.mode is None:
        self.mode = 0
      if self.current is None:
        self.current = ff_msgs.msg.ControlState()
      if self.next is None:
        self.next = ff_msgs.msg.ControlState()
    else:
      self.header = std_msgs.msg.Header()
      self.mode = 0
      self.current = ff_msgs.msg.ControlState()
      self.next = ff_msgs.msg.ControlState()

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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_B2I19d2I19d().pack(_x.mode, _x.current.when.secs, _x.current.when.nsecs, _x.current.pose.position.x, _x.current.pose.position.y, _x.current.pose.position.z, _x.current.pose.orientation.x, _x.current.pose.orientation.y, _x.current.pose.orientation.z, _x.current.pose.orientation.w, _x.current.twist.linear.x, _x.current.twist.linear.y, _x.current.twist.linear.z, _x.current.twist.angular.x, _x.current.twist.angular.y, _x.current.twist.angular.z, _x.current.accel.linear.x, _x.current.accel.linear.y, _x.current.accel.linear.z, _x.current.accel.angular.x, _x.current.accel.angular.y, _x.current.accel.angular.z, _x.next.when.secs, _x.next.when.nsecs, _x.next.pose.position.x, _x.next.pose.position.y, _x.next.pose.position.z, _x.next.pose.orientation.x, _x.next.pose.orientation.y, _x.next.pose.orientation.z, _x.next.pose.orientation.w, _x.next.twist.linear.x, _x.next.twist.linear.y, _x.next.twist.linear.z, _x.next.twist.angular.x, _x.next.twist.angular.y, _x.next.twist.angular.z, _x.next.accel.linear.x, _x.next.accel.linear.y, _x.next.accel.linear.z, _x.next.accel.angular.x, _x.next.accel.angular.y, _x.next.accel.angular.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.current is None:
        self.current = ff_msgs.msg.ControlState()
      if self.next is None:
        self.next = ff_msgs.msg.ControlState()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 321
      (_x.mode, _x.current.when.secs, _x.current.when.nsecs, _x.current.pose.position.x, _x.current.pose.position.y, _x.current.pose.position.z, _x.current.pose.orientation.x, _x.current.pose.orientation.y, _x.current.pose.orientation.z, _x.current.pose.orientation.w, _x.current.twist.linear.x, _x.current.twist.linear.y, _x.current.twist.linear.z, _x.current.twist.angular.x, _x.current.twist.angular.y, _x.current.twist.angular.z, _x.current.accel.linear.x, _x.current.accel.linear.y, _x.current.accel.linear.z, _x.current.accel.angular.x, _x.current.accel.angular.y, _x.current.accel.angular.z, _x.next.when.secs, _x.next.when.nsecs, _x.next.pose.position.x, _x.next.pose.position.y, _x.next.pose.position.z, _x.next.pose.orientation.x, _x.next.pose.orientation.y, _x.next.pose.orientation.z, _x.next.pose.orientation.w, _x.next.twist.linear.x, _x.next.twist.linear.y, _x.next.twist.linear.z, _x.next.twist.angular.x, _x.next.twist.angular.y, _x.next.twist.angular.z, _x.next.accel.linear.x, _x.next.accel.linear.y, _x.next.accel.linear.z, _x.next.accel.angular.x, _x.next.accel.angular.y, _x.next.accel.angular.z,) = _get_struct_B2I19d2I19d().unpack(str[start:end])
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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_B2I19d2I19d().pack(_x.mode, _x.current.when.secs, _x.current.when.nsecs, _x.current.pose.position.x, _x.current.pose.position.y, _x.current.pose.position.z, _x.current.pose.orientation.x, _x.current.pose.orientation.y, _x.current.pose.orientation.z, _x.current.pose.orientation.w, _x.current.twist.linear.x, _x.current.twist.linear.y, _x.current.twist.linear.z, _x.current.twist.angular.x, _x.current.twist.angular.y, _x.current.twist.angular.z, _x.current.accel.linear.x, _x.current.accel.linear.y, _x.current.accel.linear.z, _x.current.accel.angular.x, _x.current.accel.angular.y, _x.current.accel.angular.z, _x.next.when.secs, _x.next.when.nsecs, _x.next.pose.position.x, _x.next.pose.position.y, _x.next.pose.position.z, _x.next.pose.orientation.x, _x.next.pose.orientation.y, _x.next.pose.orientation.z, _x.next.pose.orientation.w, _x.next.twist.linear.x, _x.next.twist.linear.y, _x.next.twist.linear.z, _x.next.twist.angular.x, _x.next.twist.angular.y, _x.next.twist.angular.z, _x.next.accel.linear.x, _x.next.accel.linear.y, _x.next.accel.linear.z, _x.next.accel.angular.x, _x.next.accel.angular.y, _x.next.accel.angular.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.current is None:
        self.current = ff_msgs.msg.ControlState()
      if self.next is None:
        self.next = ff_msgs.msg.ControlState()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 321
      (_x.mode, _x.current.when.secs, _x.current.when.nsecs, _x.current.pose.position.x, _x.current.pose.position.y, _x.current.pose.position.z, _x.current.pose.orientation.x, _x.current.pose.orientation.y, _x.current.pose.orientation.z, _x.current.pose.orientation.w, _x.current.twist.linear.x, _x.current.twist.linear.y, _x.current.twist.linear.z, _x.current.twist.angular.x, _x.current.twist.angular.y, _x.current.twist.angular.z, _x.current.accel.linear.x, _x.current.accel.linear.y, _x.current.accel.linear.z, _x.current.accel.angular.x, _x.current.accel.angular.y, _x.current.accel.angular.z, _x.next.when.secs, _x.next.when.nsecs, _x.next.pose.position.x, _x.next.pose.position.y, _x.next.pose.position.z, _x.next.pose.orientation.x, _x.next.pose.orientation.y, _x.next.pose.orientation.z, _x.next.pose.orientation.w, _x.next.twist.linear.x, _x.next.twist.linear.y, _x.next.twist.linear.z, _x.next.twist.angular.x, _x.next.twist.angular.y, _x.next.twist.angular.z, _x.next.accel.linear.x, _x.next.accel.linear.y, _x.next.accel.linear.z, _x.next.accel.angular.x, _x.next.accel.angular.y, _x.next.accel.angular.z,) = _get_struct_B2I19d2I19d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_B2I19d2I19d = None
def _get_struct_B2I19d2I19d():
    global _struct_B2I19d2I19d
    if _struct_B2I19d2I19d is None:
        _struct_B2I19d2I19d = struct.Struct("<B2I19d2I19d")
    return _struct_B2I19d2I19d
