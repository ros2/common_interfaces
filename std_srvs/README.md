# std_srvs

This package provides several service definitions for standard but simple ROS services.

For more information about ROS 2 interfaces, see [docs.ros.org](https://docs.ros.org/en/rolling/Concepts/About-ROS-Interfaces.html).

## Services (.srv)
* [Empty.srv](srv/Empty.srv): A service containing an empty request and response.
* [SetBool.srv](srv/SetBool.srv): Service to set a boolean state to true or false, for enabling or disabling hardware for example.
* [SetFloat32.srv](srv/SetFloat32.srv): Service to set a float32 value, for setting a numeric threshold or gain for example.
* [SetInt32.srv](srv/SetInt32.srv): Service to set an int32 value, for setting an integer mode or counter for example.
* [SetString.srv](srv/SetString.srv): Service to set a string value, for selecting a profile or a configuration name for example.
* [SetUInt8.srv](srv/SetUInt8.srv): Service to set an unsigned 8-bit integer value, for setting a small command code for example.
* [SetUInt8Array.srv](srv/SetUInt8Array.srv): Service to set an array of unsigned 8-bit integer values, for sending a compact payload for example.
* [Trigger.srv](srv/Trigger.srv): Service with an empty request header used for triggering the activation or start of a service.


## Quality Declaration
This package claims to be in the **Quality Level 1** category, see the [Quality Declaration](QUALITY_DECLARATION.md) for more details.
