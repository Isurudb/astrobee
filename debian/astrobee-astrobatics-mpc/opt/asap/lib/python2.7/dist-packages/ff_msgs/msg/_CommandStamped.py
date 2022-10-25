# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from ff_msgs/CommandStamped.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import ff_msgs.msg
import std_msgs.msg

class CommandStamped(genpy.Message):
  _md5sum = "ac01350894b1be9e3a7b4f390a14812d"
  _type = "ff_msgs/CommandStamped"
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
# Command Message, loosely based off of the RAPID Command.idl

# Header with timestamp
std_msgs/Header header

# Command name
string cmd_name

# Unique identifier for command = unique counter + participant + timestamp
string cmd_id

# Source of the command, either operators, the system monitor or guest science
string cmd_src

# Origin of the command, ground for operators, astrobee for another astrobee,
# sys_monitor for fault responses, and guest_science for guest science
# commands
string cmd_origin

# Name of subsystem the command is going to (not used but kept to be consistant
# with the command idl)
string subsys_name

# Arguments for the command 
ff_msgs/CommandArg[] args

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
MSG: ff_msgs/CommandArg
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
# An argument to a command sent through RAPID
#
# Note that this is approximating a union in DDS. However, this is an
# inefficient union, and thus each instance will take up at least 89 bytes.
# However, even with the maximum of 16 arguments to a command, we only have
# about 1k extra data. I, tfmorse, am ok with that. Commands are rarely sent.

uint8 DATA_TYPE_BOOL     = 0
uint8 DATA_TYPE_DOUBLE   = 1
uint8 DATA_TYPE_FLOAT    = 2
uint8 DATA_TYPE_INT      = 3
uint8 DATA_TYPE_LONGLONG = 4
uint8 DATA_TYPE_STRING   = 5
uint8 DATA_TYPE_VEC3d    = 6
uint8 DATA_TYPE_MAT33f   = 7

uint8 data_type

bool b
float64 d
float32 f
int32 i
int64 ll
string s
float64[3] vec3d
float32[9] mat33f

"""
  __slots__ = ['header','cmd_name','cmd_id','cmd_src','cmd_origin','subsys_name','args']
  _slot_types = ['std_msgs/Header','string','string','string','string','string','ff_msgs/CommandArg[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,cmd_name,cmd_id,cmd_src,cmd_origin,subsys_name,args

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(CommandStamped, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.cmd_name is None:
        self.cmd_name = ''
      if self.cmd_id is None:
        self.cmd_id = ''
      if self.cmd_src is None:
        self.cmd_src = ''
      if self.cmd_origin is None:
        self.cmd_origin = ''
      if self.subsys_name is None:
        self.subsys_name = ''
      if self.args is None:
        self.args = []
    else:
      self.header = std_msgs.msg.Header()
      self.cmd_name = ''
      self.cmd_id = ''
      self.cmd_src = ''
      self.cmd_origin = ''
      self.subsys_name = ''
      self.args = []

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
      _x = self.cmd_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.cmd_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.cmd_src
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.cmd_origin
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.subsys_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.args)
      buff.write(_struct_I.pack(length))
      for val1 in self.args:
        _x = val1
        buff.write(_get_struct_2Bdfiq().pack(_x.data_type, _x.b, _x.d, _x.f, _x.i, _x.ll))
        _x = val1.s
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        buff.write(_get_struct_3d().pack(*val1.vec3d))
        buff.write(_get_struct_9f().pack(*val1.mat33f))
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
      if self.args is None:
        self.args = None
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
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.cmd_name = str[start:end].decode('utf-8')
      else:
        self.cmd_name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.cmd_id = str[start:end].decode('utf-8')
      else:
        self.cmd_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.cmd_src = str[start:end].decode('utf-8')
      else:
        self.cmd_src = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.cmd_origin = str[start:end].decode('utf-8')
      else:
        self.cmd_origin = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.subsys_name = str[start:end].decode('utf-8')
      else:
        self.subsys_name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.args = []
      for i in range(0, length):
        val1 = ff_msgs.msg.CommandArg()
        _x = val1
        start = end
        end += 26
        (_x.data_type, _x.b, _x.d, _x.f, _x.i, _x.ll,) = _get_struct_2Bdfiq().unpack(str[start:end])
        val1.b = bool(val1.b)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.s = str[start:end].decode('utf-8')
        else:
          val1.s = str[start:end]
        start = end
        end += 24
        val1.vec3d = _get_struct_3d().unpack(str[start:end])
        start = end
        end += 36
        val1.mat33f = _get_struct_9f().unpack(str[start:end])
        self.args.append(val1)
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
      _x = self.cmd_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.cmd_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.cmd_src
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.cmd_origin
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.subsys_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.args)
      buff.write(_struct_I.pack(length))
      for val1 in self.args:
        _x = val1
        buff.write(_get_struct_2Bdfiq().pack(_x.data_type, _x.b, _x.d, _x.f, _x.i, _x.ll))
        _x = val1.s
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        buff.write(val1.vec3d.tostring())
        buff.write(val1.mat33f.tostring())
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
      if self.args is None:
        self.args = None
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
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.cmd_name = str[start:end].decode('utf-8')
      else:
        self.cmd_name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.cmd_id = str[start:end].decode('utf-8')
      else:
        self.cmd_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.cmd_src = str[start:end].decode('utf-8')
      else:
        self.cmd_src = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.cmd_origin = str[start:end].decode('utf-8')
      else:
        self.cmd_origin = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.subsys_name = str[start:end].decode('utf-8')
      else:
        self.subsys_name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.args = []
      for i in range(0, length):
        val1 = ff_msgs.msg.CommandArg()
        _x = val1
        start = end
        end += 26
        (_x.data_type, _x.b, _x.d, _x.f, _x.i, _x.ll,) = _get_struct_2Bdfiq().unpack(str[start:end])
        val1.b = bool(val1.b)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.s = str[start:end].decode('utf-8')
        else:
          val1.s = str[start:end]
        start = end
        end += 24
        val1.vec3d = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=3)
        start = end
        end += 36
        val1.mat33f = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=9)
        self.args.append(val1)
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
_struct_9f = None
def _get_struct_9f():
    global _struct_9f
    if _struct_9f is None:
        _struct_9f = struct.Struct("<9f")
    return _struct_9f
_struct_2Bdfiq = None
def _get_struct_2Bdfiq():
    global _struct_2Bdfiq
    if _struct_2Bdfiq is None:
        _struct_2Bdfiq = struct.Struct("<2Bdfiq")
    return _struct_2Bdfiq
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d
