This document is a declaration of software quality for the `diagnostic_msgs` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# `diagnostic_msgs` Quality Declaration

The package `diagnostic_msgs` claims to be in the **Quality Level 4** category.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Requirements for Quality Level 4 in REP-2004](https://www.ros.org/reps/rep-2004.html).

## Version Policy [1]

### Version Scheme [1.i]

`diagnostic_msgs` uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#versioning).

### Version Stability [1.ii]

`diagnostic_msgs` is at a stable version, i.e. `>= 1.0.0`.
The current version can be found in its [package.xml](package.xml), and its change history can be found in its [CHANGELOG](CHANGELOG.rst).

### Public API Declaration [1.iii]

All message and service definition files located in `msg` and `srv` directories are considered part of the public API.

### API Stability Within a Released ROS Distribution [1.iv]/[1.vi]

`diagnostic_msgs` will not break public API within a released ROS distribution, i.e. no major releases once the ROS distribution is released.

### ABI Stability Within a Released ROS Distribution [1.v]/[1.vi]

`diagnostic_msgs` does not contain any C or C++ code and therefore will not affect ABI stability.

## Change Control Process [2]

`diagnostic_msgs` follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#package-requirements).

### Change Requests [2.i]

This package requires that all changes occur through a pull request.

### Contributor Origin [2.ii]

This package uses DCO as its confirmation of contributor origin policy. More information can be found in [CONTRIBUTING](../CONTRIBUTING.md).

### Peer Review Policy [2.iii]

Following the recommended guidelines for ROS Core packages, all pull requests must have at least 1 peer review.

### Continuous Integration [2.iv]

All pull requests must pass CI on all [tier 1 platforms](https://www.ros.org/reps/rep-2000.html#support-tiers)

### Documentation Policy [2.v]

All pull requests must resolve related documentation changes before merging.

## Documentation

### Feature Documentation [3.i]

`diagnostic_msgs` has a list of provided [messages and services](README.md).
New messages and services require their own documentation in order to be added.

### Public API Documentation [3.ii]

`diagnostic_msgs` has embedded API documentation, but it is not currently hosted.

### License [3.iii]

The license for `diagnostic_msgs` is Apache 2.0, the type is declared in the [package.xml](package.xml) manifest file, and a full copy of the license is in the repository level [LICENSE](../LICENSE) file.

There are no source files that are currently copyrighted in this package so files are not checked for abbreviated license statements.

### Copyright Statements [3.iv]

There are no currently copyrighted source files in this package.

## Testing [4]

`diagnostic_msgs` is a package providing strictly message and service definitions and therefore does not require associated tests and has no coverage or performance requirements.

### Linters and Static Analysis [4.v]

`diagnostic_msgs` uses and passes all the standard linters and static analysis tools for its generated C++ and Python code to ensure it follows the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#linters).

Results of the nightly linter tests can be found [here](http://build.ros2.org/view/Epr/job/Epr__common_interfaces__ubuntu_bionic_amd64/lastBuild/testReport/diagnostic_msgs/).

## Dependencies [5]

### Direct Runtime ROS Dependencies [5.i]/[5.ii]

`diagnostic_msgs` has the following runtime ROS dependencies:
* `builtin_interfaces`
* `geometry_msgs`
* `rosidl_default_runtime`
* `std_msgs`

It has several "buildtool" dependencies, which do not affect the resulting quality of the package, because they do not contribute to the public library API.

### Direct Runtime Non-ROS Dependencies [5.iii]

`diagnostic_msgs` does not have any runtime non-ROS dependencies.

## Platform Support [6]

`diagnostic_msgs` supports all of the tier 1 platforms as described in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers), and tests each change against all of them.

Currently nightly results can be seen here:
* [linux-aarch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastBuild/testReport/diagnostic_msgs/)
* [linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/diagnostic_msgs/)
* [mac_osx_release](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastBuild/testReport/diagnostic_msgs/)
* [windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastBuild/testReport/diagnostic_msgs/)

## Vulnerability Disclosure Policy [7.i]

This package does not yet have a Vulnerability Disclosure Policy
