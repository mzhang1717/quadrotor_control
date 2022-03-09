"""autogenerated by genpy from quadrotor_input/SelectStateSourceRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class SelectStateSourceRequest(genpy.Message):
  _md5sum = "210a52aa3c6055403a0fb5920bce78a8"
  _type = "quadrotor_input/SelectStateSourceRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """string strPoseTopicSelected
string strVelocityTopicSelected

"""
  __slots__ = ['strPoseTopicSelected','strVelocityTopicSelected']
  _slot_types = ['string','string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       strPoseTopicSelected,strVelocityTopicSelected

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SelectStateSourceRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.strPoseTopicSelected is None:
        self.strPoseTopicSelected = ''
      if self.strVelocityTopicSelected is None:
        self.strVelocityTopicSelected = ''
    else:
      self.strPoseTopicSelected = ''
      self.strVelocityTopicSelected = ''

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
      _x = self.strPoseTopicSelected
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.strVelocityTopicSelected
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

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
        self.strPoseTopicSelected = str[start:end].decode('utf-8')
      else:
        self.strPoseTopicSelected = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.strVelocityTopicSelected = str[start:end].decode('utf-8')
      else:
        self.strVelocityTopicSelected = str[start:end]
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
      _x = self.strPoseTopicSelected
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.strVelocityTopicSelected
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
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
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.strPoseTopicSelected = str[start:end].decode('utf-8')
      else:
        self.strPoseTopicSelected = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.strVelocityTopicSelected = str[start:end].decode('utf-8')
      else:
        self.strVelocityTopicSelected = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
"""autogenerated by genpy from quadrotor_input/SelectStateSourceResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class SelectStateSourceResponse(genpy.Message):
  _md5sum = "1633c17a5b2ed98c236852d24a2d1588"
  _type = "quadrotor_input/SelectStateSourceResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """bool bSuccess


"""
  __slots__ = ['bSuccess']
  _slot_types = ['bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       bSuccess

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SelectStateSourceResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.bSuccess is None:
        self.bSuccess = False
    else:
      self.bSuccess = False

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
      buff.write(_struct_B.pack(self.bSuccess))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 1
      (self.bSuccess,) = _struct_B.unpack(str[start:end])
      self.bSuccess = bool(self.bSuccess)
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
      buff.write(_struct_B.pack(self.bSuccess))
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
      start = end
      end += 1
      (self.bSuccess,) = _struct_B.unpack(str[start:end])
      self.bSuccess = bool(self.bSuccess)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_B = struct.Struct("<B")
class SelectStateSource(object):
  _type          = 'quadrotor_input/SelectStateSource'
  _md5sum = '11317bd1430c644f8910ccd389a7161d'
  _request_class  = SelectStateSourceRequest
  _response_class = SelectStateSourceResponse