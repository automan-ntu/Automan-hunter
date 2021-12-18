# generated from rosidl_generator_py/resource/_idl.py.em
# with input from hunter_msgs:msg/HunterStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_HunterStatus(type):
    """Metaclass of message 'HunterStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'MOTOR_ID_FRONT': 0,
        'MOTOR_ID_REAR_LEFT': 1,
        'MOTOR_ID_REAR_RIGHT': 2,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('hunter_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'hunter_msgs.msg.HunterStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__hunter_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__hunter_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__hunter_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__hunter_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__hunter_status

            from hunter_msgs.msg import HunterMotorState
            if HunterMotorState.__class__._TYPE_SUPPORT is None:
                HunterMotorState.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'MOTOR_ID_FRONT': cls.__constants['MOTOR_ID_FRONT'],
            'MOTOR_ID_REAR_LEFT': cls.__constants['MOTOR_ID_REAR_LEFT'],
            'MOTOR_ID_REAR_RIGHT': cls.__constants['MOTOR_ID_REAR_RIGHT'],
        }

    @property
    def MOTOR_ID_FRONT(self):
        """Message constant 'MOTOR_ID_FRONT'."""
        return Metaclass_HunterStatus.__constants['MOTOR_ID_FRONT']

    @property
    def MOTOR_ID_REAR_LEFT(self):
        """Message constant 'MOTOR_ID_REAR_LEFT'."""
        return Metaclass_HunterStatus.__constants['MOTOR_ID_REAR_LEFT']

    @property
    def MOTOR_ID_REAR_RIGHT(self):
        """Message constant 'MOTOR_ID_REAR_RIGHT'."""
        return Metaclass_HunterStatus.__constants['MOTOR_ID_REAR_RIGHT']


class HunterStatus(metaclass=Metaclass_HunterStatus):
    """
    Message class 'HunterStatus'.

    Constants:
      MOTOR_ID_FRONT
      MOTOR_ID_REAR_LEFT
      MOTOR_ID_REAR_RIGHT
    """

    __slots__ = [
        '_header',
        '_linear_velocity',
        '_steering_angle',
        '_base_state',
        '_control_mode',
        '_fault_code',
        '_battery_voltage',
        '_motor_states',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'linear_velocity': 'double',
        'steering_angle': 'double',
        'base_state': 'uint8',
        'control_mode': 'uint8',
        'fault_code': 'uint16',
        'battery_voltage': 'double',
        'motor_states': 'hunter_msgs/HunterMotorState[3]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.NamespacedType(['hunter_msgs', 'msg'], 'HunterMotorState'), 3),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.linear_velocity = kwargs.get('linear_velocity', float())
        self.steering_angle = kwargs.get('steering_angle', float())
        self.base_state = kwargs.get('base_state', int())
        self.control_mode = kwargs.get('control_mode', int())
        self.fault_code = kwargs.get('fault_code', int())
        self.battery_voltage = kwargs.get('battery_voltage', float())
        from hunter_msgs.msg import HunterMotorState
        self.motor_states = kwargs.get(
            'motor_states',
            [HunterMotorState() for x in range(3)]
        )

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.linear_velocity != other.linear_velocity:
            return False
        if self.steering_angle != other.steering_angle:
            return False
        if self.base_state != other.base_state:
            return False
        if self.control_mode != other.control_mode:
            return False
        if self.fault_code != other.fault_code:
            return False
        if self.battery_voltage != other.battery_voltage:
            return False
        if self.motor_states != other.motor_states:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @property
    def linear_velocity(self):
        """Message field 'linear_velocity'."""
        return self._linear_velocity

    @linear_velocity.setter
    def linear_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'linear_velocity' field must be of type 'float'"
        self._linear_velocity = value

    @property
    def steering_angle(self):
        """Message field 'steering_angle'."""
        return self._steering_angle

    @steering_angle.setter
    def steering_angle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'steering_angle' field must be of type 'float'"
        self._steering_angle = value

    @property
    def base_state(self):
        """Message field 'base_state'."""
        return self._base_state

    @base_state.setter
    def base_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'base_state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'base_state' field must be an unsigned integer in [0, 255]"
        self._base_state = value

    @property
    def control_mode(self):
        """Message field 'control_mode'."""
        return self._control_mode

    @control_mode.setter
    def control_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'control_mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'control_mode' field must be an unsigned integer in [0, 255]"
        self._control_mode = value

    @property
    def fault_code(self):
        """Message field 'fault_code'."""
        return self._fault_code

    @fault_code.setter
    def fault_code(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'fault_code' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'fault_code' field must be an unsigned integer in [0, 65535]"
        self._fault_code = value

    @property
    def battery_voltage(self):
        """Message field 'battery_voltage'."""
        return self._battery_voltage

    @battery_voltage.setter
    def battery_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'battery_voltage' field must be of type 'float'"
        self._battery_voltage = value

    @property
    def motor_states(self):
        """Message field 'motor_states'."""
        return self._motor_states

    @motor_states.setter
    def motor_states(self, value):
        if __debug__:
            from hunter_msgs.msg import HunterMotorState
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, HunterMotorState) for v in value) and
                 True), \
                "The 'motor_states' field must be a set or sequence with length 3 and each value of type 'HunterMotorState'"
        self._motor_states = value
