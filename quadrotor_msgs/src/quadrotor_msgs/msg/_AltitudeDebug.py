"""autogenerated by genpy from quadrotor_msgs/AltitudeDebug.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class AltitudeDebug(genpy.Message):
  _md5sum = "0eeff40074d8bc6fbac2ac754822d6ba"
  _type = "quadrotor_msgs/AltitudeDebug"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64 p_error
float64 i_error
float64 d_error
float64 cmd
float64 cmd_limited
float64 z
float64 z_des
float64 base_throttle
float64 throttle
float64 scale
float64 voltage

"""
  __slots__ = ['p_error','i_error','d_error','cmd','cmd_limited','z','z_des','base_throttle','throttle','scale','voltage']
  _slot_types = ['float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       p_error,i_error,d_error,cmd,cmd_limited,z,z_des,base_throttle,throttle,scale,voltage

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(AltitudeDebug, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.p_error is None:
        self.p_error = 0.
      if self.i_error is None:
        self.i_error = 0.
      if self.d_error is None:
        self.d_error = 0.
      if self.cmd is None:
        self.cmd = 0.
      if self.cmd_limited is None:
        self.cmd_limited = 0.
      if self.z is None:
        self.z = 0.
      if self.z_des is None:
        self.z_des = 0.
      if self.base_throttle is None:
        self.base_throttle = 0.
      if self.throttle is None:
        self.throttle = 0.
      if self.scale is None:
        self.scale = 0.
      if self.voltage is None:
        self.voltage = 0.
    else:
      self.p_error = 0.
      self.i_error = 0.
      self.d_error = 0.
      self.cmd = 0.
      self.cmd_limited = 0.
      self.z = 0.
      self.z_des = 0.
      self.base_throttle = 0.
      self.throttle = 0.
      self.scale = 0.
      self.voltage = 0.

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
      buff.write(_struct_11d.pack(_x.p_error, _x.i_error, _x.d_error, _x.cmd, _x.cmd_limited, _x.z, _x.z_des, _x.base_throttle, _x.throttle, _x.scale, _x.voltage))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 88
      (_x.p_error, _x.i_error, _x.d_error, _x.cmd, _x.cmd_limited, _x.z, _x.z_des, _x.base_throttle, _x.throttle, _x.scale, _x.voltage,) = _struct_11d.unpack(str[start:end])
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
      buff.write(_struct_11d.pack(_x.p_error, _x.i_error, _x.d_error, _x.cmd, _x.cmd_limited, _x.z, _x.z_des, _x.base_throttle, _x.throttle, _x.scale, _x.voltage))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 88
      (_x.p_error, _x.i_error, _x.d_error, _x.cmd, _x.cmd_limited, _x.z, _x.z_des, _x.base_throttle, _x.throttle, _x.scale, _x.voltage,) = _struct_11d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_11d = struct.Struct("<11d")
