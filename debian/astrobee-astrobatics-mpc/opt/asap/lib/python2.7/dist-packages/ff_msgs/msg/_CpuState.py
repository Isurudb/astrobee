# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from ff_msgs/CpuState.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class CpuState(genpy.Message):
  _md5sum = "3ff6c0a8b78ea1e9087461c1e42b6ca2"
  _type = "ff_msgs/CpuState"
  _has_header = False #flag to mark the presence of a Header object
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
# State of a CPU.

# Processor is on (enabled) or not
bool enabled

# The load (in percentages) of the cpu, for the fields given in
# CpuStateStamped
float32[] loads 

# Current operating frequency in Hz
uint32 frequency

# Max frequency (may be less than theoretical limit of the processor)
uint32 max_frequency
"""
  __slots__ = ['enabled','loads','frequency','max_frequency']
  _slot_types = ['bool','float32[]','uint32','uint32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       enabled,loads,frequency,max_frequency

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(CpuState, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.enabled is None:
        self.enabled = False
      if self.loads is None:
        self.loads = []
      if self.frequency is None:
        self.frequency = 0
      if self.max_frequency is None:
        self.max_frequency = 0
    else:
      self.enabled = False
      self.loads = []
      self.frequency = 0
      self.max_frequency = 0

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
      buff.write(_get_struct_B().pack(self.enabled))
      length = len(self.loads)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.loads))
      _x = self
      buff.write(_get_struct_2I().pack(_x.frequency, _x.max_frequency))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 1
      (self.enabled,) = _get_struct_B().unpack(str[start:end])
      self.enabled = bool(self.enabled)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.loads = struct.unpack(pattern, str[start:end])
      _x = self
      start = end
      end += 8
      (_x.frequency, _x.max_frequency,) = _get_struct_2I().unpack(str[start:end])
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
      buff.write(_get_struct_B().pack(self.enabled))
      length = len(self.loads)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.loads.tostring())
      _x = self
      buff.write(_get_struct_2I().pack(_x.frequency, _x.max_frequency))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 1
      (self.enabled,) = _get_struct_B().unpack(str[start:end])
      self.enabled = bool(self.enabled)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.loads = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      _x = self
      start = end
      end += 8
      (_x.frequency, _x.max_frequency,) = _get_struct_2I().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
_struct_2I = None
def _get_struct_2I():
    global _struct_2I
    if _struct_2I is None:
        _struct_2I = struct.Struct("<2I")
    return _struct_2I
