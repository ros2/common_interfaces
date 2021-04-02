This document is a declaration of software quality for the `sensor_msgs` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# `sensor_msgs` Quality Declaration

The package `sensor_msgs` claims to be in the **Quality Level 1** category as long as it is used with a **Quality Level 1** middleware.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Requirements for Quality Level 1 in REP-2004](https://www.ros.org/reps/rep-2004.html).

## Version Policy [1]

### Version Scheme [1.i]

`sensor_msgs` uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://docs.ros.org/en/foxy/Contributing/Developer-Guide.html#versioning).

### Version Stability [1.ii]

`sensor_msgs` is at a stable version, i.e. `>= 1.0.0`.
The current version can be found in its [package.xml](package.xml), and its change history can be found in its [CHANGELOG](CHANGELOG.rst).

### Public API Declaration [1.iii]

All message and service definition files located in `include`, `msg` and `srv` directories are considered part of the public API.

### API Stability Within a Released ROS Distribution [1.iv]/[1.vi]

`sensor_msgs` will not break public API within a released ROS distribution, i.e. no major releases once the ROS distribution is released.

### ABI Stability Within a Released ROS Distribution [1.v]/[1.vi]

`sensor_msgs` contains C++ code and is therefore concerned with ABI stability. It will not break ABI stability within a released ROS distribution.

## Change Control Process [2]

`sensor_msgs` follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://docs.ros.org/en/foxy/Contributing/Developer-Guide.html#quality-practices).

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

`sensor_msgs` has a list of provided [messages and services](README.md).
New messages and services require their own documentation in order to be added.

### Public API Documentation [3.ii]

`sensor_msgs` has embedded API documentation, but it is not currently hosted.

### License [3.iii]

The license for `sensor_msgs` is Apache 2.0, the type is declared in the [package.xml](package.xml) manifest file, and a full copy of the license is in the repository level [LICENSE](../LICENSE) file.

There is an automated test which runs a linter that ensures each file has a license statement.

Most recent test results can be found [here](http://build.ros2.org/view/Fpr/job/Fpr__common_interfaces__ubuntu_focal_amd64/lastCompletedBuild/testReport/sensor_msgs/copyright/)

### Copyright Statements [3.iv]

There are no currently copyrighted source files in this package.

## Testing [4]

### Feature Testing [4.i]

Most of the features in sensor_msgs have corresponding tests which simulate typical usage, and they are located in the `test` directory.
New features are required to have tests before being added.

Results of these feature tests can be found [here](http://build.ros2.org/view/Fpr/job/Fpr__common_interfaces__ubuntu_focal_amd64/lastCompletedBuild/testReport/(root)/sensor_msgs/)

### Public API Testing [4.ii]

Each part of the public non-generated C++ API has tests, and new additions or changes to the public API require tests before being added.
The tests aim to cover both typical usage and corner cases, but are quantified by contributing to code coverage.

Results of these API tests can be found [here](http://build.ros2.org/view/Fpr/job/Fpr__common_interfaces__ubuntu_focal_amd64/lastCompletedBuild/testReport/(root)/sensor_msgs/)

### Coverage [4.iii]

`sensor_msgs` follows the recommendations for ROS Core packages in the [ROS 2 Developer Guide](https://docs.ros.org/en/foxy/Contributing/Developer-Guide.html#code-coverage), and opts to use line coverage instead of branch coverage.

This includes:

- tracking and reporting line coverage statistics
- achieving and maintaining a reasonable branch line coverage (90-100%)
- no lines are manually skipped in coverage calculations

Changes are required to make a best effort to keep or increase coverage before being accepted, but decreases are allowed if properly justified and accepted by reviewers.

Current coverage statistics can be viewed [here](https://ci.ros2.org/job/nightly_linux_foxy_coverage/lastCompletedBuild/cobertura).
A description of how coverage statistics are summarized from this page, can be found in the ["ROS 2 Onboarding Guide"](https://docs.ros.org/en/foxy/Contributing/Developer-Guide.html#note-on-coverage-runs).

### Performance [4.iv]

`sensor_msgs` follows the recommendations for performance testing of C code in the [ROS 2 Developer Guide](https://docs.ros.org/en/foxy/Contributing/Developer-Guide.html#performance), and opts to do performance analysis on each release rather than each change.

System level performance benchmarks that cover features of `sensor_msgs` can be found at:
* [Benchmarks](http://build.ros2.org/view/Fci/job/Fci__benchmark_ubuntu_focal_amd64/BenchmarkTable/)
* [Performance](http://build.ros2.org/view/Fci/job/Fci__nightly-performance_ubuntu_focal_amd64/lastCompletedBuild/)

Changes that introduce regressions in performance must be adequately justified in order to be accepted and merged.

### Linters and Static Analysis [4.v]

`sensor_msgs` uses and passes all the standard linters and static analysis tools for its generated C++ and Python code to ensure it follows the [ROS 2 Developer Guide](https://docs.ros.org/en/foxy/Contributing/Developer-Guide.html#linters-and-static-analysis).

Results of the nightly linter tests can be found [here](http://build.ros2.org/view/Fpr/job/Fpr__common_interfaces__ubuntu_focal_amd64/lastCompletedBuild/testReport/sensor_msgs/).

## Dependencies [5]

### Direct Runtime ROS Dependencies [5.i]/[5.ii]

`sensor_msgs` has the following runtime ROS dependencies, which are at **Quality Level 1**:
* `builtin_interfaces`: [QUALITY DECLARATION](https://github.com/ros2/rcl_interfaces/tree/foxy/builtin_interfaces/QUALITY_DECLARATION.md)
* `geometry_msgs`: [QUALITY DECLARATION](../geometry_msgs/QUALITY_DECLARATION.md)
* `rosidl_default_runtime` [QUALITY DECLARATION](https://github.com/ros2/rosidl_defaults/tree/foxy/rosidl_default_runtime/QUALITY_DECLARATION.md)
* `std_msgs`: [QUALITY DECLARATION](../std_msgs/QUALITY_DECLARATION.md)

It has several "buildtool" dependencies, which do not affect the resulting quality of the package, because they do not contribute to the public library API.

### Direct Runtime Non-ROS Dependencies [5.iii]

`sensor_msgs` does not have any runtime non-ROS dependencies.

## Platform Support [6]

`sensor_msgs` supports all of the tier 1 platforms as described in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers), and tests each change against all of them.

Though there are no nightly jobs for foxy outside of linux, each change is tested on ci.ros2.org.
* [linux-aarch64](https://ci.ros2.org/job/ci_linux-aarch64)
* [linux](https://ci.ros2.org/job/ci_linux)
* [mac_osx](https://ci.ros2.org/job/ci_osx)
* [windows](https://ci.ros2.org/job/ci_windows)

## Vulnerability Disclosure Policy [7.i]

This package conforms to the Vulnerability Disclosure Policy in [REP-2006](https://www.ros.org/reps/rep-2006.html).
