# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from ff_msgs/DepthOdometry.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import ff_msgs.msg
import geometry_msgs.msg
import genpy
import std_msgs.msg

class DepthOdometry(genpy.Message):
  _md5sum = "d00049c091a5ccf31e3eef01d010e9fa"
  _type = "ff_msgs/DepthOdometry"
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

Header header
ff_msgs/Odometry odometry
ff_msgs/DepthCorrespondence[] correspondences
bool valid_image_points
bool valid_points_3d
float32 runtime

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
MSG: ff_msgs/Odometry
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

time source_time 
time target_time
geometry_msgs/PoseWithCovariance sensor_F_source_T_target
geometry_msgs/PoseWithCovariance body_F_source_T_target

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

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
MSG: ff_msgs/DepthCorrespondence
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
ImagePoint source_image_point
ImagePoint target_image_point
geometry_msgs/Point source_3d_point
geometry_msgs/Point target_3d_point

================================================================================
MSG: ff_msgs/ImagePoint
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

float32 x
float32 y
"""
  __slots__ = ['header','odometry','correspondences','valid_image_points','valid_points_3d','runtime']
  _slot_types = ['std_msgs/Header','ff_msgs/Odometry','ff_msgs/DepthCorrespondence[]','bool','bool','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,odometry,correspondences,valid_image_points,valid_points_3d,runtime

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(DepthOdometry, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.odometry is None:
        self.odometry = ff_msgs.msg.Odometry()
      if self.correspondences is None:
        self.correspondences = []
      if self.valid_image_points is None:
        self.valid_image_points = False
      if self.valid_points_3d is None:
        self.valid_points_3d = False
      if self.runtime is None:
        self.runtime = 0.
    else:
      self.header = std_msgs.msg.Header()
      self.odometry = ff_msgs.msg.Odometry()
      self.correspondences = []
      self.valid_image_points = False
      self.valid_points_3d = False
      self.runtime = 0.

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
      buff.write(_get_struct_4I7d().pack(_x.odometry.source_time.secs, _x.odometry.source_time.nsecs, _x.odometry.target_time.secs, _x.odometry.target_time.nsecs, _x.odometry.sensor_F_source_T_target.pose.position.x, _x.odometry.sensor_F_source_T_target.pose.position.y, _x.odometry.sensor_F_source_T_target.pose.position.z, _x.odometry.sensor_F_source_T_target.pose.orientation.x, _x.odometry.sensor_F_source_T_target.pose.orientation.y, _x.odometry.sensor_F_source_T_target.pose.orientation.z, _x.odometry.sensor_F_source_T_target.pose.orientation.w))
      buff.write(_get_struct_36d().pack(*self.odometry.sensor_F_source_T_target.covariance))
      _x = self
      buff.write(_get_struct_7d().pack(_x.odometry.body_F_source_T_target.pose.position.x, _x.odometry.body_F_source_T_target.pose.position.y, _x.odometry.body_F_source_T_target.pose.position.z, _x.odometry.body_F_source_T_target.pose.orientation.x, _x.odometry.body_F_source_T_target.pose.orientation.y, _x.odometry.body_F_source_T_target.pose.orientation.z, _x.odometry.body_F_source_T_target.pose.orientation.w))
      buff.write(_get_struct_36d().pack(*self.odometry.body_F_source_T_target.covariance))
      length = len(self.correspondences)
      buff.write(_struct_I.pack(length))
      for val1 in self.correspondences:
        _v1 = val1.source_image_point
        _x = _v1
        buff.write(_get_struct_2f().pack(_x.x, _x.y))
        _v2 = val1.target_image_point
        _x = _v2
        buff.write(_get_struct_2f().pack(_x.x, _x.y))
        _v3 = val1.source_3d_point
        _x = _v3
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v4 = val1.target_3d_point
        _x = _v4
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
      _x = self
      buff.write(_get_struct_2Bf().pack(_x.valid_image_points, _x.valid_points_3d, _x.runtime))
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
      if self.odometry is None:
        self.odometry = ff_msgs.msg.Odometry()
      if self.correspondences is None:
        self.correspondences = None
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
      end += 72
      (_x.odometry.source_time.secs, _x.odometry.source_time.nsecs, _x.odometry.target_time.secs, _x.odometry.target_time.nsecs, _x.odometry.sensor_F_source_T_target.pose.position.x, _x.odometry.sensor_F_source_T_target.pose.position.y, _x.odometry.sensor_F_source_T_target.pose.position.z, _x.odometry.sensor_F_source_T_target.pose.orientation.x, _x.odometry.sensor_F_source_T_target.pose.orientation.y, _x.odometry.sensor_F_source_T_target.pose.orientation.z, _x.odometry.sensor_F_source_T_target.pose.orientation.w,) = _get_struct_4I7d().unpack(str[start:end])
      start = end
      end += 288
      self.odometry.sensor_F_source_T_target.covariance = _get_struct_36d().unpack(str[start:end])
      _x = self
      start = end
      end += 56
      (_x.odometry.body_F_source_T_target.pose.position.x, _x.odometry.body_F_source_T_target.pose.position.y, _x.odometry.body_F_source_T_target.pose.position.z, _x.odometry.body_F_source_T_target.pose.orientation.x, _x.odometry.body_F_source_T_target.pose.orientation.y, _x.odometry.body_F_source_T_target.pose.orientation.z, _x.odometry.body_F_source_T_target.pose.orientation.w,) = _get_struct_7d().unpack(str[start:end])
      start = end
      end += 288
      self.odometry.body_F_source_T_target.covariance = _get_struct_36d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.correspondences = []
      for i in range(0, length):
        val1 = ff_msgs.msg.DepthCorrespondence()
        _v5 = val1.source_image_point
        _x = _v5
        start = end
        end += 8
        (_x.x, _x.y,) = _get_struct_2f().unpack(str[start:end])
        _v6 = val1.target_image_point
        _x = _v6
        start = end
        end += 8
        (_x.x, _x.y,) = _get_struct_2f().unpack(str[start:end])
        _v7 = val1.source_3d_point
        _x = _v7
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v8 = val1.target_3d_point
        _x = _v8
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        self.correspondences.append(val1)
      _x = self
      start = end
      end += 6
      (_x.valid_image_points, _x.valid_points_3d, _x.runtime,) = _get_struct_2Bf().unpack(str[start:end])
      self.valid_image_points = bool(self.valid_image_points)
      self.valid_points_3d = bool(self.valid_points_3d)
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
      buff.write(_get_struct_4I7d().pack(_x.odometry.source_time.secs, _x.odometry.source_time.nsecs, _x.odometry.target_time.secs, _x.odometry.target_time.nsecs, _x.odometry.sensor_F_source_T_target.pose.position.x, _x.odometry.sensor_F_source_T_target.pose.position.y, _x.odometry.sensor_F_source_T_target.pose.position.z, _x.odometry.sensor_F_source_T_target.pose.orientation.x, _x.odometry.sensor_F_source_T_target.pose.orientation.y, _x.odometry.sensor_F_source_T_target.pose.orientation.z, _x.odometry.sensor_F_source_T_target.pose.orientation.w))
      buff.write(self.odometry.sensor_F_source_T_target.covariance.tostring())
      _x = self
      buff.write(_get_struct_7d().pack(_x.odometry.body_F_source_T_target.pose.position.x, _x.odometry.body_F_source_T_target.pose.position.y, _x.odometry.body_F_source_T_target.pose.position.z, _x.odometry.body_F_source_T_target.pose.orientation.x, _x.odometry.body_F_source_T_target.pose.orientation.y, _x.odometry.body_F_source_T_target.pose.orientation.z, _x.odometry.body_F_source_T_target.pose.orientation.w))
      buff.write(self.odometry.body_F_source_T_target.covariance.tostring())
      length = len(self.correspondences)
      buff.write(_struct_I.pack(length))
      for val1 in self.correspondences:
        _v9 = val1.source_image_point
        _x = _v9
        buff.write(_get_struct_2f().pack(_x.x, _x.y))
        _v10 = val1.target_image_point
        _x = _v10
        buff.write(_get_struct_2f().pack(_x.x, _x.y))
        _v11 = val1.source_3d_point
        _x = _v11
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v12 = val1.target_3d_point
        _x = _v12
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
      _x = self
      buff.write(_get_struct_2Bf().pack(_x.valid_image_points, _x.valid_points_3d, _x.runtime))
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
      if self.odometry is None:
        self.odometry = ff_msgs.msg.Odometry()
      if self.correspondences is None:
        self.correspondences = None
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
      end += 72
      (_x.odometry.source_time.secs, _x.odometry.source_time.nsecs, _x.odometry.target_time.secs, _x.odometry.target_time.nsecs, _x.odometry.sensor_F_source_T_target.pose.position.x, _x.odometry.sensor_F_source_T_target.pose.position.y, _x.odometry.sensor_F_source_T_target.pose.position.z, _x.odometry.sensor_F_source_T_target.pose.orientation.x, _x.odometry.sensor_F_source_T_target.pose.orientation.y, _x.odometry.sensor_F_source_T_target.pose.orientation.z, _x.odometry.sensor_F_source_T_target.pose.orientation.w,) = _get_struct_4I7d().unpack(str[start:end])
      start = end
      end += 288
      self.odometry.sensor_F_source_T_target.covariance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=36)
      _x = self
      start = end
      end += 56
      (_x.odometry.body_F_source_T_target.pose.position.x, _x.odometry.body_F_source_T_target.pose.position.y, _x.odometry.body_F_source_T_target.pose.position.z, _x.odometry.body_F_source_T_target.pose.orientation.x, _x.odometry.body_F_source_T_target.pose.orientation.y, _x.odometry.body_F_source_T_target.pose.orientation.z, _x.odometry.body_F_source_T_target.pose.orientation.w,) = _get_struct_7d().unpack(str[start:end])
      start = end
      end += 288
      self.odometry.body_F_source_T_target.covariance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=36)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.correspondences = []
      for i in range(0, length):
        val1 = ff_msgs.msg.DepthCorrespondence()
        _v13 = val1.source_image_point
        _x = _v13
        start = end
        end += 8
        (_x.x, _x.y,) = _get_struct_2f().unpack(str[start:end])
        _v14 = val1.target_image_point
        _x = _v14
        start = end
        end += 8
        (_x.x, _x.y,) = _get_struct_2f().unpack(str[start:end])
        _v15 = val1.source_3d_point
        _x = _v15
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v16 = val1.target_3d_point
        _x = _v16
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        self.correspondences.append(val1)
      _x = self
      start = end
      end += 6
      (_x.valid_image_points, _x.valid_points_3d, _x.runtime,) = _get_struct_2Bf().unpack(str[start:end])
      self.valid_image_points = bool(self.valid_image_points)
      self.valid_points_3d = bool(self.valid_points_3d)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_7d = None
def _get_struct_7d():
    global _struct_7d
    if _struct_7d is None:
        _struct_7d = struct.Struct("<7d")
    return _struct_7d
_struct_36d = None
def _get_struct_36d():
    global _struct_36d
    if _struct_36d is None:
        _struct_36d = struct.Struct("<36d")
    return _struct_36d
_struct_2f = None
def _get_struct_2f():
    global _struct_2f
    if _struct_2f is None:
        _struct_2f = struct.Struct("<2f")
    return _struct_2f
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_4I7d = None
def _get_struct_4I7d():
    global _struct_4I7d
    if _struct_4I7d is None:
        _struct_4I7d = struct.Struct("<4I7d")
    return _struct_4I7d
_struct_2Bf = None
def _get_struct_2Bf():
    global _struct_2Bf
    if _struct_2Bf is None:
        _struct_2Bf = struct.Struct("<2Bf")
    return _struct_2Bf
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d
