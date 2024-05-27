# common_interfaces
A set of packages which contain common interface files (.msg and .srv).


## Purpose

Isolating the messages to communicate between stacks in a shared dependency allows nodes in dependent stacks to communicate without requiring dependencies upon each other.
This repository has been designed to contain the most common messages used between multiple packages to provide a shared dependency which will eliminate a problematic circular dependency.

## Contributing 

For how to contribute see [CONTRIBUTING.md](common_interfaces/CONTRIBUTING.md)
