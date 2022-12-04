# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from ff_msgs/SaveSettings.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class SaveSettings(genpy.Message):
  _md5sum = "87300656673b0987cb5b546a70fa697f"
  _type = "ff_msgs/SaveSettings"
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

# The save settings message contains information about the topics currently
# being recorded.

# Name of topic
string topic_name

# Topic saved to disk; upon docking it is downlinked
uint8 IMMEDIATE   = 0

# Topic saved to disk; upon docking it is transferred to ISS server for later
# downlink
uint8 DELAYED     = 1

# Downlink option indicates if and when the data in the rostopic is downlinked
uint8 downlinkOption

# Times per second to save the data (Hz)
float32 frequency
"""
  # Pseudo-constants
  IMMEDIATE = 0
  DELAYED = 1

  __slots__ = ['topic_name','downlinkOption','frequency']
  _slot_types = ['string','uint8','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       topic_name,downlinkOption,frequency

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SaveSettings, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.topic_name is None:
        self.topic_name = ''
      if self.downlinkOption is None:
        self.downlinkOption = 0
      if self.frequency is None:
        self.frequency = 0.
    else:
      self.topic_name = ''
      self.downlinkOption = 0
      self.frequency = 0.

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
      _x = self.topic_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_Bf().pack(_x.downlinkOption, _x.frequency))
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
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.topic_name = str[start:end].decode('utf-8')
      else:
        self.topic_name = str[start:end]
      _x = self
      start = end
      end += 5
      (_x.downlinkOption, _x.frequency,) = _get_struct_Bf().unpack(str[start:end])
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
      _x = self.topic_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_Bf().pack(_x.downlinkOption, _x.frequency))
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
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.topic_name = str[start:end].decode('utf-8')
      else:
        self.topic_name = str[start:end]
      _x = self
      start = end
      end += 5
      (_x.downlinkOption, _x.frequency,) = _get_struct_Bf().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_Bf = None
def _get_struct_Bf():
    global _struct_Bf
    if _struct_Bf is None:
        _struct_Bf = struct.Struct("<Bf")
    return _struct_Bf