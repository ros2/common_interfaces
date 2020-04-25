# std_msgs

`std_msgs` provides many basic message types. Only a few messages are intended for incorporation into higher-level messages. The primitive and primitive array types should generally not be relied upon for long-term use.

For more information about ROS 2 interfaces, see [index.ros2.org](https://index.ros.org/doc/ros2/Concepts/About-ROS-Interfaces/)

## Messages (.msg)
* [ColorRGBA](ColorRGBA.msg): A single RGBA value for representing colors.
* [Empty](Empty.msg): Does not hold any information, useful when the sending of a message would provide sufficient information.
* [Header](Header.msg): Standard metadata for higher-level stamped data types used to communicate timestamped data in a particular coordinate frame.

### Primitive Types
`std_msgs` provides the following wrappers for ROS primitive types, which are documented in the msg specification. It also contains the Empty type, which is useful for sending an empty signal. However, these types do not convey semantic meaning about their contents: every message simply has a field called "data". Therefore, while the messages in this package can be useful for quick prototyping, they are NOT intended for "long-term" usage. For ease of documentation and collaboration, we recommend that existing messages be used, or new messages created, that provide meaningful field name(s).

* [Bool](Bool.msg)
* [Byte](Byte.msg)
* [Char](Char.msg)
* [Float32](Float32.msg)
* [Float64](Float64.msg)
* [Int8](Int8.msg)
* [Int16](Int16.msg)
* [Int32](Int32.msg)
* [Int64](Int64.msg)
* [String](String.msg)
* [UInt8](UInt8.msg)
* [UInt16](UInt16.msg)
* [UInt32](UInt32.msg)
* [UInt64](UInt64.msg)

### Array Types
`std_msgs` also provides the following "MultiArray" types, which can be useful for storing sensor data. However, the same caveat as above applies: it's usually "better" (in the sense of making the code easier to understand, etc.) when developers use or create non-generic message types (see discussion in this thread for more detail).

* [ByteMultiArray](ByteMultiArray.msg)
* [Float32MultiArray](Float32MultiArray.msg)
* [Float64MultiArray](Float64MultiArray.msg)
* [Int8MultiArray](Int8MultiArray.msg)
* [Int16MultiArray](Int16MultiArray.msg)
* [Int32MultiArray](Int32MultiArray.msg)
* [Int64MultiArray](Int64MultiArray.msg)
* [MultiArrayDimension](MultiArrayDimension.msg)
* [MultiArrayLayout](MultiArrayLayout.msg)
* [UInt16MultiArray](UInt16MultiArray.msg)
* [UInt32MultiArray](UInt32MultiArray.msg)
* [UInt64MultiArray](UInt64MultiArray.msg)
* [UInt8MultiArray](UInt8MultiArray.msg)

## Quality Declaration
This package claims to be in the **Quality Level 4** category, see the [Quality Declaration](QUALITY_DECLARATION.md) for more details.
