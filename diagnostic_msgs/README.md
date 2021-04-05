# diagnostic_msgs

This package provides several messages and services for ROS node diagnostics.

For more information about ROS 2 interfaces, see [docs.ros.org](https://docs.ros.org/en/rolling/Concepts/About-ROS-Interfaces.html).

## Messages (.msg)
* [DiagnosticArray](msg/DiagnosticArray.msg): Used to send diagnostic information about the state of the robot.
* [DiagnosticStatus](msg/DiagnosticStatus.msg): Holds the status of an individual component of the robot.
* [KeyValue](msg/KeyValue.msg): Associates diagnostic values with their labels.

## Services (.srv)
* [AddDiagnostics](srv/AddDiagnostics.srv): Used as part of the process for loading analyzers at runtime, not for use as a standalone service.
* [SelfTest](srv/SelfTest.srv): Call this service to perform a diagnostic check.

## Quality Declaration
This package claims to be in the **Quality Level 1** category, see the [Quality Declaration](QUALITY_DECLARATION.md) for more details.
