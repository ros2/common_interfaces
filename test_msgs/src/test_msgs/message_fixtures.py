# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from test_msgs.msg import BoundedArrayNested
from test_msgs.msg import BoundedArrayPrimitives
from test_msgs.msg import Builtins
from test_msgs.msg import DynamicArrayNested
from test_msgs.msg import DynamicArrayPrimitives
from test_msgs.msg import Empty
from test_msgs.msg import FieldsWithSameType
from test_msgs.msg import Nested
from test_msgs.msg import Primitives
from test_msgs.msg import StaticArrayNested
from test_msgs.msg import StaticArrayPrimitives


def get_msg_builtins():
    msg = Builtins()
    msg.duration_value.sec = -1234567890
    msg.duration_value.nanosec = 123456789
    msg.time_value.sec = -1234567890
    msg.time_value.nanosec = 987654321

    return [msg]


def get_msg_empty():
    msg = Empty()

    return [msg]


def get_msg_primitives():
    msgs = []
    msg = Primitives()
    msg.bool_value = False
    msg.byte_value = bytes([0])
    msg.char_value = '\x00'
    msg.float32_value = float(0.0)
    msg.float64_value = float(0.0)
    msg.int8_value = 0
    msg.uint8_value = 0
    msg.int16_value = 0
    msg.uint16_value = 0
    msg.int32_value = 0
    msg.uint32_value = 0
    msg.int64_value = 0
    msg.uint64_value = 0
    msg.string_value = ''
    msgs.append(msg)

    msg = Primitives()
    msg.bool_value = True
    msg.byte_value = bytes([255])
    msg.char_value = '\x7f'
    msg.float32_value = 1.125
    msg.float64_value = 1.125
    msg.int8_value = 127
    msg.uint8_value = 255
    msg.int16_value = 32767
    msg.uint16_value = 65535
    msg.int32_value = 2147483647
    msg.uint32_value = 4294967295
    msg.int64_value = 9223372036854775807
    msg.uint64_value = 18446744073709551615
    msg.string_value = 'max value'
    msgs.append(msg)

    msg = Primitives()
    msg.bool_value = False
    msg.byte_value = bytes([0])
    msg.char_value = '\x00'
    msg.float32_value = -2.125
    msg.float64_value = -2.125
    msg.int8_value = -128
    msg.uint8_value = 0
    msg.int16_value = -32768
    msg.uint16_value = 0
    msg.int32_value = -2147483648
    msg.uint32_value = 0
    msg.int64_value = -9223372036854775808
    msg.uint64_value = 0
    msg.string_value = 'min value'
    msgs.append(msg)

    msg = Primitives()
    msg.bool_value = True
    msg.byte_value = bytes([1])
    msg.char_value = '\x01'
    msg.float32_value = float(1.0)
    msg.float64_value = float(1.0)
    msg.int8_value = 1
    msg.uint8_value = 1
    msg.int16_value = 1
    msg.uint16_value = 1
    msg.int32_value = 1
    msg.uint32_value = 1
    msg.int64_value = 1
    msg.uint64_value = 1
    msg.string_value = ''
    # check strings longer then 255 characters
    for i in range(20000):
        msg.string_value += str(i % 10)
    msgs.append(msg)

    return msgs


def get_msg_fields_with_same_type():
    msg = FieldsWithSameType()
    msg.primitive_values1 = get_msg_primitives()[0]
    msg.primitive_values2 = get_msg_primitives()[0]

    return [msg]


def get_msg_nested():
    msgs = []
    msg = Nested()

    primitive_msgs = get_msg_primitives()
    for primitive_msg in primitive_msgs:
        msg = Nested()
        msg.primitive_values = primitive_msg
        msgs.append(msg)

    return msgs


def get_msg_static_array_primitives():
    msgs = []
    msg = StaticArrayPrimitives()
    msg.bool_values = [False, True, False]
    msg.char_values = ['\0', '\x7f', '\x00']
    msg.byte_values = [bytes([0]), bytes([255]), bytes([0])]
    msg.float32_values = [0.0, 1.125, -2.125]
    msg.float64_values = [0.0, 1.125, -2.125]
    msg.int8_values = [0, 127, -128]
    msg.uint8_values = [0, 255, 0]
    msg.int16_values = [0, 32767, -32768]
    msg.uint16_values = [0, 65535, 0]
    msg.int32_values = [0, 2147483647, -2147483648]
    msg.uint32_values = [0, 4294967295, 0]
    msg.int64_values = [0, 9223372036854775807, -9223372036854775808]
    msg.uint64_values = [0, 18446744073709551615, 0]
    msg.string_values = ['', 'max value', 'min value']
    msgs.append(msg)

    return msgs


def get_msg_static_array_nested():
    msg = StaticArrayNested()
    primitive_msgs = get_msg_primitives()
    assert len(primitive_msgs) == len(msg.primitive_values)
    i = 0
    for primitive_msg in primitive_msgs:
        msg.primitive_values[i] = primitive_msg
        i += 1

    return [msg]


def get_msg_dynamic_array_primitives():
    msgs = []
    msg = DynamicArrayPrimitives()
    msg.bool_values = []
    msg.char_values = []
    msg.byte_values = []
    msg.float32_values = []
    msg.float64_values = []
    msg.int8_values = []
    msg.uint8_values = []
    msg.int16_values = []
    msg.uint16_values = []
    msg.int32_values = []
    msg.uint32_values = []
    msg.int64_values = []
    msg.uint64_values = []
    msg.string_values = []
    msg.check = 0
    msgs.append(msg)

    msg = DynamicArrayPrimitives()
    msg.bool_values = [True]
    msg.byte_values = [bytes([255])]
    msg.char_values = ['\x7f']
    msg.float32_values = [1.125]
    msg.float64_values = [1.125]
    msg.int8_values = [127]
    msg.uint8_values = [255]
    msg.int16_values = [32767]
    msg.uint16_values = [65535]
    msg.int32_values = [2147483647]
    msg.uint32_values = [4294967295]
    msg.int64_values = [9223372036854775807]
    msg.uint64_values = [18446744073709551615]
    msg.string_values = ['max value']
    msg.check = 1
    msgs.append(msg)

    msg = DynamicArrayPrimitives()
    msg.bool_values = [False, True]
    msg.byte_values = [bytes([0]), bytes([255])]
    msg.char_values = ['\0', '\x7f']
    msg.float32_values = [0.0, 1.125, -2.125]
    msg.float64_values = [0.0, 1.125, -2.125]
    msg.int8_values = [0, 127, -128]
    msg.uint8_values = [0, 255]
    msg.int16_values = [0, 32767, -32768]
    msg.uint16_values = [0, 65535]
    msg.int32_values = [0, 2147483647, -2147483648]
    msg.uint32_values = [0, 4294967295]
    msg.int64_values = [0, 9223372036854775807, -9223372036854775808]
    msg.uint64_values = [0, 18446744073709551615]
    msg.string_values = ['', 'max value', 'optional min value']
    msg.check = 2
    msgs.append(msg)

    size = 101
    msg = DynamicArrayPrimitives()
    msg.bool_values = [i % 2 != 0 for i in range(size)]
    msg.byte_values = [bytes([i]) for i in range(size)]
    msg.char_values = [chr(i) for i in range(size)]
    msg.float32_values = [float(1.125 * i) for i in range(size)]
    msg.float64_values = [1.125 * i for i in range(size)]
    msg.int8_values = [i for i in range(size)]
    msg.uint8_values = [i for i in range(size)]
    msg.int16_values = [i for i in range(size)]
    msg.uint16_values = [i for i in range(size)]
    msg.int32_values = [i for i in range(size)]
    msg.uint32_values = [i for i in range(size)]
    msg.int64_values = [i for i in range(size)]
    msg.uint64_values = [i for i in range(size)]
    msg.string_values = [str(i) for i in range(size)]
    msg.check = 3
    msgs.append(msg)

    msg = DynamicArrayPrimitives()
    msg.check = 4
    msgs.append(msg)

    return msgs


def get_msg_dynamic_array_nested():
    msg = DynamicArrayNested()
    for primitive_msg in get_msg_primitives():
        msg.primitive_values.append(primitive_msg)

    return [msg]


def get_msg_bounded_array_primitives():
    msgs = []

    msg = BoundedArrayPrimitives()
    msg.bool_values = [False, True, False]
    msg.byte_values = [bytes([0]), bytes([1]), bytes([255])]
    msg.char_values = ['\0', '\1', '\x7f']
    msg.float32_values = [0.0, 1.125, -2.125]
    msg.float64_values = [0.0, 1.125, -2.125]
    msg.int8_values = [0, 127, -128]
    msg.uint8_values = [0, 1, 255]
    msg.int16_values = [0, 32767, -32768]
    msg.uint16_values = [0, 1, 65535]
    msg.int32_values = [0, 2147483647, -2147483648]
    msg.uint32_values = [0, 1, 4294967295]
    msg.int64_values = [0, 9223372036854775807, -9223372036854775808]
    msg.uint64_values = [0, 1, 18446744073709551615]
    msg.string_values = ['', 'max value', 'optional min value']
    msg.check = 2
    msgs.append(msg)

    msg = BoundedArrayPrimitives()
    msg.check = 4
    msgs.append(msg)

    return msgs


def get_msg_bounded_array_nested():
    msg = BoundedArrayNested()
    for primitive_msg in get_msg_primitives():
        msg.primitive_values.append(primitive_msg)

    return [msg]


def get_test_msg(message_name):
    if 'Builtins' == message_name:
        msg = get_msg_builtins()
    elif 'Empty' == message_name:
        msg = get_msg_empty()
    elif 'Primitives' == message_name:
        msg = get_msg_primitives()
    elif 'Nested' == message_name:
        msg = get_msg_nested()
    elif 'FieldsWithSameYype' == message_name:
        msg = get_msg_fields_with_same_type()
    elif 'StaticArrayNested' == message_name:
        msg = get_msg_static_array_nested()
    elif 'StaticArrayPrimitives' == message_name:
        msg = get_msg_static_array_primitives()
    elif 'DynamicArrayPrimitives' == message_name:
        msg = get_msg_dynamic_array_primitives()
    elif 'DynamicArrayNested' == message_name:
        msg = get_msg_dynamic_array_nested()
    elif 'BoundedArrayPrimitives' == message_name:
        msg = get_msg_bounded_array_primitives()
    elif 'BoundedArrayNested' == message_name:
        msg = get_msg_bounded_array_nested()
    else:
        raise NotImplementedError
    return msg
