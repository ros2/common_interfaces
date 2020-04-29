# std_msgs

`std_msgs` provides many basic message types. Only a few messages are intended for incorporation into higher-level messages. The primitive and primitive array types should generally not be relied upon for long-term use.

For more information about ROS 2 interfaces, see [index.ros2.org](https://index.ros.org/doc/ros2/Concepts/About-ROS-Interfaces/)

## Messages (.msg)
* [ColorRGBA](msg/ColorRGBA.msg): A single RGBA value for representing colors.
* [Empty](msg/Empty.msg): Does not hold any information, useful when the sending of a message would provide sufficient information.
* [Header](msg/Header.msg): Standard metadata for higher-level stamped data types used to communicate timestamped data in a particular coordinate frame.

### Primitive Types
`std_msgs` provides the following wrappers for ROS primitive types, which are documented in the msg specification. It also contains the Empty type, which is useful for sending an empty signal. However, these types do not convey semantic meaning about their contents: every message simply has a field called "data". Therefore, while the messages in this package can be useful for quick prototyping, they are NOT intended for "long-term" usage. For ease of documentation and collaboration, we recommend that existing messages be used, or new messages created, that provide meaningful field name(s).

* [Bool](msg/Bool.msg)
* [Byte](msg/Byte.msg)
* [Char](msg/Char.msg)
* [Float32](msg/Float32.msg)
* [Float64](msg/Float64.msg)
* [Int8](msg/Int8.msg)
* [Int16](msg/Int16.msg)
* [Int32](msg/Int32.msg)
* [Int64](msg/Int64.msg)
* [String](msg/String.msg)
* [UInt8](msg/UInt8.msg)
* [UInt16](msg/UInt16.msg)
* [UInt32](msg/UInt32.msg)
* [UInt64](msg/UInt64.msg)

### Array Types
`std_msgs` also provides the following "MultiArray" types, which can be useful for storing sensor data. However, the same caveat as above applies: it's usually "better" (in the sense of making the code easier to understand, etc.) when developers use or create non-generic message types (see discussion in this thread for more detail).

* [ByteMultiArray](msg/ByteMultiArray.msg)
* [Float32MultiArray](msg/Float32MultiArray.msg)
* [Float64MultiArray](msg/Float64MultiArray.msg)
* [Int8MultiArray](msg/Int8MultiArray.msg)
* [Int16MultiArray](msg/Int16MultiArray.msg)
* [Int32MultiArray](msg/Int32MultiArray.msg)
* [Int64MultiArray](msg/Int64MultiArray.msg)
* [MultiArrayDimension](msg/MultiArrayDimension.msg)
* [MultiArrayLayout](msg/MultiArrayLayout.msg)
* [UInt16MultiArray](msg/UInt16MultiArray.msg)
* [UInt32MultiArray](msg/UInt32MultiArray.msg)
* [UInt64MultiArray](msg/UInt64MultiArray.msg)
* [UInt8MultiArray](msg/UInt8MultiArray.msg)

## Quality Declaration
This package claims to be in the **Quality Level 4** category, see the [Quality Declaration](QUALITY_DECLARATION.md) for more details.
