# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from cartesian_planner/CenterLinePoint.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class CenterLinePoint(genpy.Message):
  _md5sum = "d8c676686a759a5bfb0f50165bdf3705"
  _type = "cartesian_planner/CenterLinePoint"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """float64 s
float64 x
float64 y
float64 theta
float64 kappa
float64 left_bound
float64 right_bound
"""
  __slots__ = ['s','x','y','theta','kappa','left_bound','right_bound']
  _slot_types = ['float64','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       s,x,y,theta,kappa,left_bound,right_bound

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(CenterLinePoint, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.s is None:
        self.s = 0.
      if self.x is None:
        self.x = 0.
      if self.y is None:
        self.y = 0.
      if self.theta is None:
        self.theta = 0.
      if self.kappa is None:
        self.kappa = 0.
      if self.left_bound is None:
        self.left_bound = 0.
      if self.right_bound is None:
        self.right_bound = 0.
    else:
      self.s = 0.
      self.x = 0.
      self.y = 0.
      self.theta = 0.
      self.kappa = 0.
      self.left_bound = 0.
      self.right_bound = 0.

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
      buff.write(_get_struct_7d().pack(_x.s, _x.x, _x.y, _x.theta, _x.kappa, _x.left_bound, _x.right_bound))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 56
      (_x.s, _x.x, _x.y, _x.theta, _x.kappa, _x.left_bound, _x.right_bound,) = _get_struct_7d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_7d().pack(_x.s, _x.x, _x.y, _x.theta, _x.kappa, _x.left_bound, _x.right_bound))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 56
      (_x.s, _x.x, _x.y, _x.theta, _x.kappa, _x.left_bound, _x.right_bound,) = _get_struct_7d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

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