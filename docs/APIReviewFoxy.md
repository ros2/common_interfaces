## API Review for Foxy Release

This document is both the working copy and will be the result of the ROS 2 Messages API review in preparation for Foxy.
It is contained in the common_interfaces repository as that's where most of the messages under discussion are, however the scope includes other packages for a general review.



### What’s the goal of this review?

In preparing for the Foxy release we are bringing up new QA standards. TODO REP LINK



Outputs we'd like to generate are:

* A list of known issues.
* Triage the list into things we would like to do before 1.0.0 (foxy) and things that could or must be done later.
* Create or improve issues so that we have properly captured these issues for future improvements.

### What is not the goal of this review?

* Achieving consensus on solutions to API problems.
  Instead we’d prefer to create well written issues that collect everything that’s known, so that conversations can be carried out in parallel on that issues.
* Fix problems with API.
  We hopefully will be able to fix some things before foxy, but that’s out-of-scope for the review.


TODO review for changes to ROS 1 and history of changes/last change




# ROS 2 Message Packages API Review


## Process

Please read through the listed APIs and if you have any comments fill them out here: 

[https://docs.google.com/forms/d/e/1FAIpQLSeNogqYD0tPe7CKBP0dpSqFmf54NzWpj8ksN8vuAq55JupyIg/viewform?usp=sf_link](https://docs.google.com/forms/d/e/1FAIpQLSeNogqYD0tPe7CKBP0dpSqFmf54NzWpj8ksN8vuAq55JupyIg/viewform?usp=sf_link)

These comments will be used as an starting point for the review.


## Repositories:

Comment: Use FQN for interface types that are members of other interfaces.
For example, inside std_msgs/msg/Header, the stamp field should have type "builtin_interfaces/msg/Time" instead of "builtin_interfaces/Time".

**Suggested Action**: Consider updating all message definitions to use this syntax for clarity.
Caveat, do we support members of non-msg types?

**Followup** This isn't necessary for now but the more complete approach might be useful for future idl usage.
This would require an update to msg format.
We'll create an issue for future consderation.

https://github.com/ros2/design/issues/277


### common_interfaces


#### actionlib_msgs

Package officially deprecated and to be removed when ros1_bridge has support for actions.
It should not be used by users.

**Suggested Action**: review deprecation and make sure it’s clear.

**Followup**: https://github.com/ros2/common_interfaces/issues/90

Msg



*   [https://github.com/ros2/common_interfaces/blob/master/actionlib_msgs/msg/GoalID.msg](https://github.com/ros2/common_interfaces/blob/master/actionlib_msgs/msg/GoalID.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/actionlib_msgs/msg/GoalStatus.msg](https://github.com/ros2/common_interfaces/blob/master/actionlib_msgs/msg/GoalStatus.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/actionlib_msgs/msg/GoalStatusArray.msg](https://github.com/ros2/common_interfaces/blob/master/actionlib_msgs/msg/GoalStatusArray.msg)


#### Diagonstic_msgs

No comments on these messages.

They are unchanged in ROS 1 [since 2014](https://github.com/ros/common_msgs/commits/jade-devel/diagnostic_msgs/msg).

Msg



*   [https://github.com/ros2/common_interfaces/blob/master/diagnostic_msgs/msg/DiagnosticArray.msg](https://github.com/ros2/common_interfaces/blob/master/diagnostic_msgs/msg/DiagnosticArray.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/diagnostic_msgs/msg/DiagnosticStatus.msg](https://github.com/ros2/common_interfaces/blob/master/diagnostic_msgs/msg/DiagnosticStatus.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/diagnostic_msgs/msg/KeyValue.msg](https://github.com/ros2/common_interfaces/blob/master/diagnostic_msgs/msg/KeyValue.msg)

Srv



*   [https://github.com/ros2/common_interfaces/blob/master/diagnostic_msgs/srv/AddDiagnostics.srv](https://github.com/ros2/common_interfaces/blob/master/diagnostic_msgs/srv/AddDiagnostics.srv)
*   [https://github.com/ros2/common_interfaces/blob/master/diagnostic_msgs/srv/SelfTest.srv](https://github.com/ros2/common_interfaces/blob/master/diagnostic_msgs/srv/SelfTest.srv)


#### geometry_msgs

Last change in ROS 1 [in 2015](https://github.com/ros/common_msgs/commits/jade-devel/geometry_msgs)

Comment: Covariance expressed as a full matrix is heavy, where upper triangular is adaquate due to symmetry.
And 64bit floats is also very heavy for the level of accuracy most applications need.
This large of a message is prohibitive for low bandwidth links such as to drones and UUVs

**Suggested Action**: Create new Covariance representation (3x3 upper triangle and 6x6 upper triangle) as well as potential helper functions.
Identify areas to leverage it and then determine a migration path.
New messages can leverage it, but existing ones will have a long migration.

**Followup** https://github.com/ros2/common_interfaces/issues/91

Msg



*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Accel.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Accel.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/AccelStamped.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/AccelStamped.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/AccelWithCovariance.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/AccelWithCovariance.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/AccelWithCovarianceStamped.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/AccelWithCovarianceStamped.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Inertia.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Inertia.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/InertiaStamped.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/InertiaStamped.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Point.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Point.msg)
    *   Comment: Point and Vector3 are the same message
    *   **Suggested Action**: None, these are the same data structure but semantically different.
    *   **Follow up**: https://github.com/ros2/common_interfaces/issues/92
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Point32.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Point32.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/PointStamped.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/PointStamped.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Polygon.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Polygon.msg)
    *   Comment: Polygon uses Point32 instead of Point.
    Would expect two message types for consistency: Polygon and Polygon32
    *   **Suggested Action**: None.
    For “completeness” we could fill out the data types.
    But in general we don’t want to have a “complete” set of messages we would rather have everyone using the same datatype.
    There are very few use cases where a polygon is large enough require 64 bit precision and this saves 50% on bandwidth.
    It’s the same compromise as is made for point clouds.
    *   **Follow up** None
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/PolygonStamped.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/PolygonStamped.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Pose.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Pose.msg)
    *   Comment: Pose and Transform seem to be the same message, except one uses `Point` and the other uses `Vector3`.
    *   **Suggested Action**: None, these are the same data structure but semantically different.
    *   **Follow up**: https://github.com/ros2/common_interfaces/issues/92
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Pose2D.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Pose2D.msg)
    *   Comment: Add the same deprecation message as in ROS 1
    *   **Suggested Action**: Forward port deprecation message.
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/PoseArray.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/PoseArray.msg)
    *   **Followup**: https://github.com/ros2/common_interfaces/issues/93
    *   Comment: Renaming to PoseArrayStamped.
    As other structures with a header, are named with the suffix Stamped.
    *   **Suggested Action**: None.
    The Stamped postfix is used for being able to have both a plain datatype and a stamped version of that datatype and maintain differentiation.
    For any more complex datatypes the header is an integral part of the data information and there’s no reason to leverage just publishing the raw data and the Stamped postfix is not necessary.
    *   **Followup**: None
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/PoseStamped.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/PoseStamped.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/PoseWithCovariance.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/PoseWithCovariance.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/PoseWithCovarianceStamped.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/PoseWithCovarianceStamped.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Quaternion.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Quaternion.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/QuaternionStamped.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/QuaternionStamped.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Transform.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Transform.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/TransformStamped.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/TransformStamped.msg)
    *   Comment: TransformStamped has an extra field for the child id frame of the transform, and the name of the package suggests that the msg only would include Transform + Header (Although for historical reasons this one might be harder to change, idk)
    *   **Suggested Action**: None.
    The stamped indicates that the datatype has a header.
    A transform is a special case as it is necessary to have more information than just the header as it’s defining the transform tree instead of referencing the transform tree.
    *   **Followup** https://github.com/ros2/common_interfaces/issues/94
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Twist.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Twist.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/TwistStamped.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/TwistStamped.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/TwistWithCovariance.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/TwistWithCovariance.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/TwistWithCovarianceStamped.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/TwistWithCovarianceStamped.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Vector3.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Vector3.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Vector3Stamped.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Vector3Stamped.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Wrench.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Wrench.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/WrenchStamped.msg](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/WrenchStamped.msg)


#### nav_msgs

Msg



*   [https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/GridCells.msg](https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/GridCells.msg)
    *   Comment: Are the points in cells the center point? It could use some documentation
    *   **Suggested Action:** Improve documentation.
    *   **Followup: https://github.com/ros2/common_interfaces/issues/95
*   [https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/MapMetaData.msg](https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/MapMetaData.msg)
    *   Comment: Is `origin` the center point of the `(0,0)` cell?
    Which direction do cell coordinates increase? Is (0,0) x,y in a right handed coordinate system such that (0,0) is the bottom right cell, or is it row,col such that (0,0) is the top left cell?
    *   **Suggested Action: **Improve documentation.
    *   **Followup: https://github.com/ros2/common_interfaces/issues/95
*   [https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/OccupancyGrid.msg](https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/OccupancyGrid.msg)
    *   Comment: probability being 0-100 doesn't take advantage of all the bits the `int8` type offers.
    0-255 where 255 is 100% probability would have slightly better resolution.
    *   **Suggested Action**: None. The message was designed to support both probabilities as well as special values like the -1 unknown value.
    Switching to a slightly higher precision would cause a lot of disruption.
    If there is a use case for needing higher precision then it would make sense to define another message with significantly higher precision, potentially something like 32bit floating point.
    But until there’s a need for that we should stick with the status quo.
    *   **Followup** https://github.com/ros2/common_interfaces/issues/96
*   [https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/Odometry.msg](https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/Odometry.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/Path.msg](https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/Path.msg)
    *   Comment: Path message is stamped, but each individual pose is also stamped.
    It's unclear what each stamp means.
    Is the stamp in each individual pose for the time the robot needs to be at that location?
    If so, the `frame_id` that's in each individual header is still redundant with the `frame_id` on the `Path` message itself.
    However, stamping each individual pose seems inflexible to the robot being delayed.
    If it's slowed down mid path, should it target the pose at the current time, or the next closest to it's current location? This seems hard to answer if the path makes a loop such that two points on the path are physically close but far apart in time.
    For a little while nav2 had it's own version of Path which had non-stamped poses, though they removed it: [https://github.com/ros-planning/navigation2/pull/1107/files#diff-2d8dabb75c11aa980f6c2629eba1e75f](https://github.com/ros-planning/navigation2/pull/1107/files#diff-2d8dabb75c11aa980f6c2629eba1e75f)
    *   **Suggested Action**: Iterate with navigation to improve documentation of semantics of timestamps.
    Note that there are multiple valid behaviors for any given path representation.
    *   **Followup** https://github.com/ros2/common_interfaces/issues/97

Srv



*   [https://github.com/ros2/common_interfaces/blob/master/nav_msgs/srv/GetMap.srv](https://github.com/ros2/common_interfaces/blob/master/nav_msgs/srv/GetMap.srv)
*   [https://github.com/ros2/common_interfaces/blob/master/nav_msgs/srv/GetPlan.srv](https://github.com/ros2/common_interfaces/blob/master/nav_msgs/srv/GetPlan.srv)
*   [https://github.com/ros2/common_interfaces/blob/master/nav_msgs/srv/SetMap.srv](https://github.com/ros2/common_interfaces/blob/master/nav_msgs/srv/SetMap.srv)
    *   Comment: Any reason in particular for the service this includes the extra initial_pose field? (in comparison to the getMap.srv that only returns a map) 
    *   **Suggested Action**: None, question.
    When you change the map out an initial pose is often necessary to reinitialize any localization algorithm.
    *   **Followup** https://github.com/ros2/common_interfaces/issues/98


#### sensor_msgs

Msg



*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/BatteryState.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/BatteryState.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CameraInfo.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CameraInfo.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/ChannelFloat32.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/ChannelFloat32.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CompressedImage.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CompressedImage.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/FluidPressure.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/FluidPressure.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Illuminance.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Illuminance.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)
    *   Comment: Image encodings would be easier to find as an enum in the message rather than a string referencing a c++ header.
    *   **Suggested Action**: Look at what constants can be moved into the message out of the header.
    *   **Followup** https://github.com/ros2/common_interfaces/issues/99
    *   Comment: In a comment is said that full size of data array is step*rows.
    But there is already rows and columns defined as height and width, not sure if there could either a little mistake in the comment description or a extra variable on the usage
    *   **Suggested Action:** Review documentation and update
    *   **Followup** https://github.com/ros2/common_interfaces/issues/100
*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Imu.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Imu.msg)
    *   Comment: Covariance matrices are given as lists.
    A separate message type for a 3x3 matrix might make it easier to write one function that turns a ROS msg matrix into a math library matrix.
    *   **Suggested Action:** Consider this with the above cobariance upper triangular matrix representation.
    Note that I’d like to see data to support the conversion hypothesis.
    My impression is that a flat list is likely more easily converted than a custom datatype.
    *   **Followup** Include this in the discussion at https://github.com/ros2/common_interfaces/issues/91
    *   Comment: Specify which frame the orientation field is relative to.
    For example an ENU, NED, or simulated frame.
    Perhaps provide an additional enum field to let the user specify which.
    *   **Suggested Action**: None.
    The data is oriented relative to the frame_id specified in the header.
    That frame maybe ENU or NED but the message doesn’t know or care.
    *   **Followup** None https://github.com/ros2/common_interfaces/pull/86/files#r392498705
*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/JointState.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/JointState.msg)
    *   Comment: JointState actually holds plural joint states
    *   **Suggested Action:** None.
    Changing this name would be quite intrusive, and it’s not a wrong statment it’s just ambiguous about how big the state vector is.
    You can look at is as the state of all joint instances.
    *   **Followup** This is being discussed in the ros_control project.
    We can take the results of that process: https://github.com/ros-controls/roadmap/blob/master/design_drafts/flexible_joint_states_msg.md
*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Joy.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Joy.msg)
    *   Comment: "We should make it clear in the Joy message what the expected values of the axes should be.
    Sort of by fiat, the most common joystick driver (https://github.com/ros2/joystick_drivers) reports the axes between 1.0 and -1.0, where 1.0 is in the ""positive"" direction (forward in X, to the left in Y), and -1.0 is in the ""negative"" direction.
    I think we should add comments explaining that the joystick driver needs to ensure this.
    *   Comment:I might suggest we change the ""buttons"" to be an array of bool, instead of int32.
    If something is continuous, it should be reported in axes, and buttons are really on or off."
    *   Comment: axes doesn't indicate bounds or meaning.
    Is it [0,1]? [-1,1]? Also, unfortunately joy_node inverts all axes such that Joy messages from it report the opposite any other tool that accesses the joystick would report.
    *   Comment: Buttons could be boolean on/off.
    If a button has any other value, it might actually be an axis.
    *   **Suggested Action:** Consider cost benefit of switching buttons to bools (review raw output)
    *   **Followup** https://github.com/ros2/common_interfaces/issues/102
    *   **Suggested Action:** Document axes ranges.
    *   **Followup** https://github.com/ros2/common_interfaces/issues/101
*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/JoyFeedback.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/JoyFeedback.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/JoyFeedbackArray.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/JoyFeedbackArray.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/LaserEcho.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/LaserEcho.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/LaserScan.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/LaserScan.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/MagneticField.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/MagneticField.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/MultiDOFJointState.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/MultiDOFJointState.msg)
    *   Comment: MultiDOFJointState is actually plural states
    *   **Suggested Action**: None see JointState plural discussion
    *   **Followup** Separate discussion: https://github.com/ros2/common_interfaces/pull/86/files#r392530673
    *   Comment: JointState.msg and MultiDOFJointState.msg have really similar names, and it’s commented in MultiDOF that they follow the same structure.
    However, the structure is not similar in terms of the internal types used.
    Maybe changing the name of one, or adding a bit of explanation extra in the comments of the later to explain this difference.
    *   **Suggested Action**: Improve documentation.
    The MultiDOFJointState represents the same information as the JointState for Multi Degreee of Freedom Joints as such the naming is specifically following the same pattern for a similar class of joints.
    *   **Followup** https://github.com/ros2/common_interfaces/issues/103
*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/MultiEchoLaserScan.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/MultiEchoLaserScan.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/NavSatFix.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/NavSatFix.msg)
    *   Comment: Consider replacing NavSatFix with gps_common/GPSFix or add some of the fields from GPSFix (the orientations specifically).
    This better supports dual antenna gps setups where an accurate heading can be provided and synced with the position fix.
    *   **Suggested Action:** Consider a extending the message with an optional orientation.
    Or consider a new extended message with orientation added.
    This message is specifically generic to all GNSS systems versus the GPS one has GPS specific data so is not suitable to be replaced.
    *   **Followup** https://github.com/ros2/common_interfaces/issues/104
*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/NavSatStatus.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/NavSatStatus.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud.msg)
    *   Comment: Can we deprecate and remove PointCloud?  I think most modern code uses PointCloud2.
    Take this with a grain of salt, though, since I haven't done any deep investigation to see how widespread use of PointCloud is.
    *   **Suggested Action**: Plan for removing PointCloud
    *   **Followup** https://github.com/ros2/common_interfaces/issues/105
*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg)
    *   Comment: If the point cloud is unordered, is `row_step` the length of `data`?
    *   Comment: point_step seems to assumes a fields for a point are adjacent.
    What if the source data actually has each channel adjacent (all x values, then all y values, then all z values, then all intensities, etc)? Does the data need to be reordered to fit in this message?
    *   **Suggested Action:** Improve documentation
    *   **Followup** https://github.com/ros2/common_interfaces/issues/106
*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointField.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointField.msg)
    *   Comment: What are the possible names of point field? I've seen x, y, z, xyz before, but I don't know of any documentation about those being standard.
    *   **Suggested Action**: Improve documentation
    *   **Followup** https://github.com/ros2/common_interfaces/issues/106
*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Range.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Range.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/RegionOfInterest.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/RegionOfInterest.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/RelativeHumidity.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/RelativeHumidity.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Temperature.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Temperature.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/TimeReference.msg](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/TimeReference.msg)

Srv



*   [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/srv/SetCameraInfo.srv](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/srv/SetCameraInfo.srv)

Implementations



*   Distortion model constants: [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/include/sensor_msgs/distortion_models.hpp](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/include/sensor_msgs/distortion_models.hpp)
*   Fill image: [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/include/sensor_msgs/fill_image.hpp](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/include/sensor_msgs/fill_image.hpp)
*   Image encodings: [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/include/sensor_msgs/image_encodings.hpp](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/include/sensor_msgs/image_encodings.hpp)
    *   Comment: There's a whole slew of image encodings that are not represented in image_encodings.hpp.
    http://fourcc.org/rgb.php and http://fourcc.org/yuv.php are pretty exhaustive.
    We don't necessarily need to expose all of those, but certainly more of the common YUV ones would be nice to have.
    *   **Suggested Action:** Review image_encodings available in opencv and other platforms and potentially expand listed enumerations.
    *   **Followup** https://github.com/ros2/common_interfaces/issues/99#issuecomment-610692212
*   Point cloud 2 iterator: [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/include/sensor_msgs/point_cloud2_iterator.hpp](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/include/sensor_msgs/point_cloud2_iterator.hpp)
*   Point cloud conversoin: [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/include/sensor_msgs/point_cloud_conversion.hpp](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/include/sensor_msgs/point_cloud_conversion.hpp)
*   Point field conversoin: [https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/include/sensor_msgs/point_field_conversion.hpp](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/include/sensor_msgs/point_field_conversion.hpp)

Proposed

 



*   sensor_msgs/Bumper
    *   Comment: Should we add a message to describe the state of the bumper?  Certainly the iRobot Create (Turtlebot1) and the kobuki (Turtlebot 2) have them, I'd assume other robots have them as well.
    Complicating factors here include bumpers with multiple sensors (iRobot Create can report at least 5 different bump "zones"), and bumpers that are based on hall-effect sensors (the iRobot Braava Jet has one of these, for instance).
    *   **Suggested Action:** Look at potential additions but I would suggest continuing to protoype in other packages before trying to promote it to sensor_msgs in an untested form.
    There is a [kobuki_msgs/BumperEvent](http://docs.ros.org/hydro/api/kobuki_msgs/html/msg/BumperEvent.html) but that won’t work for Roomba/Create.
    *   **Followup** As this is a potential new message we'll defer from reviewing and it can be proposed.

#### shape_msgs

Msg



*   [https://github.com/ros2/common_interfaces/blob/master/shape_msgs/msg/Mesh.msg](https://github.com/ros2/common_interfaces/blob/master/shape_msgs/msg/Mesh.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/shape_msgs/msg/MeshTriangle.msg](https://github.com/ros2/common_interfaces/blob/master/shape_msgs/msg/MeshTriangle.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/shape_msgs/msg/Plane.msg](https://github.com/ros2/common_interfaces/blob/master/shape_msgs/msg/Plane.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/shape_msgs/msg/SolidPrimitive.msg](https://github.com/ros2/common_interfaces/blob/master/shape_msgs/msg/SolidPrimitive.msg)


#### Std_msgs

Comment: 


    std_msgs: Often in ROS 1 we see recommendations to “not use std_msgs directly but semantically meaningful messages instead”.


    It would be good to treat differently the ones that are expected to be used (e.g. Header) from the ones that “should not be used”


    Suggestions:


    take out the non-semantically meaningful messages out of std_msgs to e.g example_interfaces, and put them in another package (or remove them completely)


    Move the widely used ones (e.g. Header) to their own package to avoid building and installing the non-semantically meaningful ones


    It would both allow to save build time + install size as well as encouraging users to not use those in their custom messages


    Some relevant past discussion [Suggestions for std_srvs](https://discourse.ros.org/t/suggestions-for-std-srvs/1079/10)

**Suggested Action:** Review moving non-semantically meaninful messages into a different package.
**Followup Issue** https://github.com/ros2/common_interfaces/issues/89

Msg



*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Bool.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Bool.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Byte.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Byte.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/ByteMultiArray.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/ByteMultiArray.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Char.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Char.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/ColorRGBA.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/ColorRGBA.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Empty.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Empty.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Float32.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Float32.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Float32MultiArray.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Float32MultiArray.msg)
    *   Comment: The application of *MultiArray messages was previously discussed and their introduction to ROS was seen as an error since they do not carry a semantic meaning compared to most of the other message types.
    I want to emphasise here, that ROS should provide a generic "Tensor" message type that can be used to exchange and log N-dimensional data.
    This is often useful for machine learning purposes, where results and intermediate representations are often represented as Tensor.
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Float64.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Float64.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Float64MultiArray.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Float64MultiArray.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Header.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Header.msg)
    *   Comment: One thing I’ve wondered about is why std_msgs/Header no longer has a sequence number.
    The sequence number has been quite helpful in the past to detect lost messages.
    It would also be interesting to add an improved message for driving robot bases.
    The message that has been used until now, geometry_msgs/Twist, is problematic because it does not have a header, and hence, you cannot detect message age and discard delayed commands.
    In the micro-ROS project, we have prototyped a replacement, see [TRVCommand](https://github.com/micro-ROS/drive_base/blob/master/drive_base_msgs/msg/TRVCommand.msg) and [CommandHeader](https://github.com/micro-ROS/drive_base/blob/master/drive_base_msgs/msg/CommandHeader.msg)  btw “TRV” stands for Translational-Rotational-Velocity.
    *   Comment: Wasn’t TwistStamped added for cases where this mattered? I’m not entirely sure whether there are semantics attached to it that would prevent it from being used here.
    I’ve not seen it used with many mobile base drivers, but it would seem to be able to address this shortcoming of Twist.
    *   Comment: There is indeed now a twist stamped, but not much of the ecosystem uses it.
    I agree wider use would be valuable, especially in absence of a sequence.
    I’m going to file a ticket for that in Navigation/related.
    It won’t be in for foxy, but I’ll add a deprecation warning and move into TwistStamped in G turtle.
    The TRV command seems limited, I think we should stick to Twist.
    It can be used for omni robots with non-forward velocities and converted to ackermann trivially.
    Its also used on some drones.
    Thanks for that comment! Edit: ticket [https://github.com/ros-planning/navigation2/issues/1594](https://github.com/ros-planning/navigation2/issues/1594)
    *   Comment: [Ticket to remove seq from Header](https://github.com/ros2/common_interfaces/pull/2)
    *   **Suggested Action**: None, this has been deprecated for years, we can finally remove it.
    There are other more appropriate mechanisms to get information about dropped packets etc from the middleware than embedding that data into the users data.
    *   **Followup** None, there's a proposed replacement at: https://github.com/ros2/rmw/issues/201 
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Int16.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Int16.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Int16MultiArray.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Int16MultiArray.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Int32.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Int32.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Int32MultiArray.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Int32MultiArray.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Int64.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Int64.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Int64MultiArray.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Int64MultiArray.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Int8.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Int8.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Int8MultiArray.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Int8MultiArray.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/MultiArrayDimension.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/MultiArrayDimension.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/MultiArrayLayout.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/MultiArrayLayout.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/String.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/String.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/UInt16.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/UInt16.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/UInt16MultiArray.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/UInt16MultiArray.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/UInt32.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/UInt32.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/UInt32MultiArray.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/UInt32MultiArray.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/UInt64.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/UInt64.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/UInt64MultiArray.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/UInt64MultiArray.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/UInt8.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/UInt8.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/UInt8MultiArray.msg](https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/UInt8MultiArray.msg)


#### std_srvs

Srv



*   [https://github.com/ros2/common_interfaces/blob/master/std_srvs/srv/Empty.srv](https://github.com/ros2/common_interfaces/blob/master/std_srvs/srv/Empty.srv)
*   [https://github.com/ros2/common_interfaces/blob/master/std_srvs/srv/SetBool.srv](https://github.com/ros2/common_interfaces/blob/master/std_srvs/srv/SetBool.srv)
*   [https://github.com/ros2/common_interfaces/blob/master/std_srvs/srv/Trigger.srv](https://github.com/ros2/common_interfaces/blob/master/std_srvs/srv/Trigger.srv)


#### stereo_msgs

Comment: In addition to DisparityImage.msg , I always wished stereo_msgs would have a message type for stereo image pairs.
Instead of a stereo camera driver having to publish two separate image topics, then having every subscriber needing to use an message synchronizer to capture the matching image pairs via matching timestamp, the driver could publish both images over a stereo image pair message.
There could be common fixed length size message for an common array of two images, or perhaps in a separate message package, a message type for collecting a dynamic number of Image.msg for multi camera rigs, like motion capture or trinary cameras for multi view reconstruction.

**Suggested Action:** None.
The design was chosen to send the parallel streams of data to avoid publishing data multiple times or requiring extra subscriptions.
The simplest use case is if you want to process one of the two stereo images.
If they’re published as a pair, you will have to receive both and then discard the entire second image which wastes a lot of resources.
On the other hand we can simply have tools that collect the parallel streams of messages and give you callbacks when you get the groupings you want without causing extra bandwidth.
**Followup** No Action.
Colorization of point clouds is another example of only one subscription.
Stick with not bundling them.


Msg



*   [https://github.com/ros2/common_interfaces/blob/master/stereo_msgs/msg/DisparityImage.msg](https://github.com/ros2/common_interfaces/blob/master/stereo_msgs/msg/DisparityImage.msg)
    *   Comment:  Header that might be removed in a future release, maybe this one?
    *   **Suggested Action:** Review removal of header.
    *   **Followup Issue** https://github.com/ros2/common_interfaces/issues/88


#### trajectory_msgs

Msg



*   [https://github.com/ros2/common_interfaces/blob/master/trajectory_msgs/msg/JointTrajectory.msg](https://github.com/ros2/common_interfaces/blob/master/trajectory_msgs/msg/JointTrajectory.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/trajectory_msgs/msg/JointTrajectoryPoint.msg](https://github.com/ros2/common_interfaces/blob/master/trajectory_msgs/msg/JointTrajectoryPoint.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/trajectory_msgs/msg/MultiDOFJointTrajectory.msg](https://github.com/ros2/common_interfaces/blob/master/trajectory_msgs/msg/MultiDOFJointTrajectory.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/trajectory_msgs/msg/MultiDOFJointTrajectoryPoint.msg](https://github.com/ros2/common_interfaces/blob/master/trajectory_msgs/msg/MultiDOFJointTrajectoryPoint.msg)


#### visualization_msgs

Msg



*   [https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/msg/ImageMarker.msg](https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/msg/ImageMarker.msg)
    *   Comment: In ImageMarker.msg, action and type are defined as int32, but their constants are defined as uint8.
    Probably action and type can be changed to uint8.
    *   **Suggested Action:** Review updating const type
    *   **Followup Issue** https://github.com/ros2/common_interfaces/pull/87
*   [https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/msg/InteractiveMarker.msg](https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/msg/InteractiveMarker.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/msg/InteractiveMarkerControl.msg](https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/msg/InteractiveMarkerControl.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/msg/InteractiveMarkerFeedback.msg](https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/msg/InteractiveMarkerFeedback.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/msg/InteractiveMarkerInit.msg](https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/msg/InteractiveMarkerInit.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/msg/InteractiveMarkerPose.msg](https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/msg/InteractiveMarkerPose.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/msg/InteractiveMarkerUpdate.msg](https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/msg/InteractiveMarkerUpdate.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/msg/Marker.msg](https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/msg/Marker.msg)
    *   Comment: In Marker.msg, action and type are defined as int32, but their constants are defined as uint8.
    Probably action and type can be changed to uint8.
    *   **Suggested Action:** Review updating const type
    *   **Followup Issue** https://github.com/ros2/common_interfaces/pull/87
*   [https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/msg/MarkerArray.msg](https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/msg/MarkerArray.msg)
*   [https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/msg/MenuEntry.msg](https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/msg/MenuEntry.msg)

Srv



*   [https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/srv/GetInteractiveMarkers.srv](https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/srv/GetInteractiveMarkers.srv)


### example_interfaces

Services



*   [https://github.com/ros2/example_interfaces/blob/master/srv/AddTwoInts.srv](https://github.com/ros2/example_interfaces/blob/master/srv/AddTwoInts.srv)

Actions



*   [https://github.com/ros2/example_interfaces/blob/master/action/Fibonacci.action](https://github.com/ros2/example_interfaces/blob/master/action/Fibonacci.action)


### rcl_interfaces


### action_msgs

Comment: Refactor the package name to be consistent with other ROS 2 interface packages: "action_interfaces"

Comment: As for the internal action servers, in the rcl interfaces it is defined GoalInfo.msg and the goal related msgs and the cancelGoal.srv, however, there is no “standard” action msg (because this depends on the type of goal for the application).
Maybe not API related, but it could be good to have a link to the ros2 actions tutorial in the rcl_interfaces/action_msgs repo.

**Suggested Action:** Consider cost/benefits of renaming
**Followup Issue** https://github.com/ros2/rcl_interfaces/issues/95

Msg



*   [https://github.com/ros2/rcl_interfaces/blob/master/action_msgs/msg/GoalInfo.msg](https://github.com/ros2/rcl_interfaces/blob/master/action_msgs/msg/GoalInfo.msg)
*   [https://github.com/ros2/rcl_interfaces/blob/master/action_msgs/msg/GoalStatus.msg](https://github.com/ros2/rcl_interfaces/blob/master/action_msgs/msg/GoalStatus.msg)
*   [https://github.com/ros2/rcl_interfaces/blob/master/action_msgs/msg/GoalStatusArray.msg](https://github.com/ros2/rcl_interfaces/blob/master/action_msgs/msg/GoalStatusArray.msg)

Srv



*   [https://github.com/ros2/rcl_interfaces/blob/master/action_msgs/srv/CancelGoal.srv](https://github.com/ros2/rcl_interfaces/blob/master/action_msgs/srv/CancelGoal.srv)


#### builtin_interfaces

Msg



*   [https://github.com/ros2/rcl_interfaces/blob/master/builtin_interfaces/msg/Duration.msg](https://github.com/ros2/rcl_interfaces/blob/master/builtin_interfaces/msg/Duration.msg)
*   [https://github.com/ros2/rcl_interfaces/blob/master/builtin_interfaces/msg/Time.msg](https://github.com/ros2/rcl_interfaces/blob/master/builtin_interfaces/msg/Time.msg)
    *   Comment: What clock is the time in reference to? System clock? ROS Clock?
    *   Comment: There is no field for the rcl_clock_type_t.
    If any clock is used, other than the default RCL_SYSTEM_TIME, then accurate comparisons cannot be made when a message is received.
    Likewise, a message sender cannot specify if the time is monotonic or not, leaving the receiver to guess.
    Suggestion: add a field to denote the clock type
    *   **Suggested Action**: Review communication of time primatives.
    The design so far has been that the only time that’s consistent across the system is system and ROS time.
    Steady time and monotonic clocks are not usually expected to be synchronized between systems and as such passing timestamps doesn’t make a lot of sense.
    But maybe there are use cases to do so.
    The time message could be extended, or a new datatypes could be added that’s differently typed to avoid accidental conversions.
    *   **Followup Issue** https://github.com/ros2/rcl_interfaces/issues/94


#### composition_interfaces

Srv



*   [https://github.com/ros2/rcl_interfaces/blob/master/composition_interfaces/srv/ListNodes.srv](https://github.com/ros2/rcl_interfaces/blob/master/composition_interfaces/srv/ListNodes.srv)
*   [https://github.com/ros2/rcl_interfaces/blob/master/composition_interfaces/srv/LoadNode.srv](https://github.com/ros2/rcl_interfaces/blob/master/composition_interfaces/srv/LoadNode.srv)
*   [https://github.com/ros2/rcl_interfaces/blob/master/composition_interfaces/srv/UnloadNode.srv](https://github.com/ros2/rcl_interfaces/blob/master/composition_interfaces/srv/UnloadNode.srv)


#### lifecycle_msgs

Msg



*   [https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/msg/State.msg](https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/msg/State.msg)
*   [https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/msg/Transition.msg](https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/msg/Transition.msg)
*   [https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/msg/TransitionDescription.msg](https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/msg/TransitionDescription.msg)
*   [https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/msg/TransitionEvent.msg](https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/msg/TransitionEvent.msg)

Srv



*   [https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/srv/ChangeState.srv](https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/srv/ChangeState.srv)
*   [https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/srv/GetAvailableStates.srv](https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/srv/GetAvailableStates.srv)
*   [https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/srv/GetAvailableTransitions.srv](https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/srv/GetAvailableTransitions.srv)
*   [https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/srv/GetState.srv](https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/srv/GetState.srv)


#### rcl_interfaces

Msg



*   [https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/FloatingPointRange.msg](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/FloatingPointRange.msg)
*   [https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/IntegerRange.msg](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/IntegerRange.msg)
*   [https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/IntraProcessMessage.msg](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/IntraProcessMessage.msg)
*   [https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/ListParametersResult.msg](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/ListParametersResult.msg)
*   [https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/Log.msg](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/Log.msg)
    *   Constants in Log.msg do not match the type of level
    *   **Suggested Action**: Improve documentation of log levels for clarity.
    *   **Followup Issue** https://github.com/ros2/rcl_interfaces/pull/93
*   [https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/Parameter.msg](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/Parameter.msg)
*   [https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/ParameterDescriptor.msg](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/ParameterDescriptor.msg)
*   [https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/ParameterEvent.msg](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/ParameterEvent.msg)
*   [https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/ParameterEventDescriptors.msg](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/ParameterEventDescriptors.msg)
    *   Flagged as unused. https://github.com/ros2/rcl_interfaces/pull/90 
    *   **Suggested Action** To be ticketed for adding implementation.
*   [https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/ParameterType.msg](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/ParameterType.msg)
*   [https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/ParameterValue.msg](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/ParameterValue.msg)
*   [https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/SetParametersResult.msg](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/SetParametersResult.msg)

Srv



*   [https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/srv/DescribeParameters.srv](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/srv/DescribeParameters.srv)
*   [https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/srv/GetParameterTypes.srv](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/srv/GetParameterTypes.srv)
*   [https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/srv/GetParameters.srv](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/srv/GetParameters.srv)
*   [https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/srv/ListParameters.srv](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/srv/ListParameters.srv)
*   [https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/srv/SetParameters.srv](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/srv/SetParameters.srv)
*   [https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/srv/SetParametersAtomically.srv](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/srv/SetParametersAtomically.srv)


#### rosgraph_msgs

Msg



*   [https://github.com/ros2/rcl_interfaces/blob/master/rosgraph_msgs/msg/Clock.msg](https://github.com/ros2/rcl_interfaces/blob/master/rosgraph_msgs/msg/Clock.msg)


#### test_msgs

N/A


### unique_identifier_msgs

Messages



*   [https://github.com/ros2/unique_identifier_msgs/blob/master/msg/UUID.msg](https://github.com/ros2/unique_identifier_msgs/blob/master/msg/UUID.msg)


### navigation_msgs


#### map_msgs

**Action** Ticket: Higher level issue that this and nav_msgs should be homogenized.
**Followup Issue** https://github.com/ros-planning/navigation_msgs/issues/18

Messages



*   [https://github.com/ros-planning/navigation_msgs/blob/jade-devel/map_msgs/msg/OccupancyGridUpdate.msg](https://github.com/ros-planning/navigation_msgs/blob/jade-devel/map_msgs/msg/OccupancyGridUpdate.msg)
*   [https://github.com/ros-planning/navigation_msgs/blob/jade-devel/map_msgs/msg/PointCloud2Update.msg](https://github.com/ros-planning/navigation_msgs/blob/jade-devel/map_msgs/msg/PointCloud2Update.msg)
*   [https://github.com/ros-planning/navigation_msgs/blob/jade-devel/map_msgs/msg/ProjectedMap.msg](https://github.com/ros-planning/navigation_msgs/blob/jade-devel/map_msgs/msg/ProjectedMap.msg)
*   [https://github.com/ros-planning/navigation_msgs/blob/jade-devel/map_msgs/msg/ProjectedMapInfo.msg](https://github.com/ros-planning/navigation_msgs/blob/jade-devel/map_msgs/msg/ProjectedMapInfo.msg)
    *   Comment: Seems to be a problem with ProjectedMapsInfo.srv Because it’s a service that receives a map and returns nothing.
    *   **Suggested Action:** Review usage and documentation for a potential change in name and extension of return type
    *   **Followup Issue** https://github.com/ros-planning/navigation_msgs/issues/16

Services



*   [https://github.com/ros-planning/navigation_msgs/blob/jade-devel/map_msgs/srv/GetMapROI.srv](https://github.com/ros-planning/navigation_msgs/blob/jade-devel/map_msgs/srv/GetMapROI.srv)
*   [https://github.com/ros-planning/navigation_msgs/blob/jade-devel/map_msgs/srv/GetPointMap.srv](https://github.com/ros-planning/navigation_msgs/blob/jade-devel/map_msgs/srv/GetPointMap.srv)
*   [https://github.com/ros-planning/navigation_msgs/blob/jade-devel/map_msgs/srv/GetPointMapROI.srv](https://github.com/ros-planning/navigation_msgs/blob/jade-devel/map_msgs/srv/GetPointMapROI.srv)
*   [https://github.com/ros-planning/navigation_msgs/blob/jade-devel/map_msgs/srv/ProjectedMapsInfo.srv](https://github.com/ros-planning/navigation_msgs/blob/jade-devel/map_msgs/srv/ProjectedMapsInfo.srv)
*   [https://github.com/ros-planning/navigation_msgs/blob/jade-devel/map_msgs/srv/SaveMap.srv](https://github.com/ros-planning/navigation_msgs/blob/jade-devel/map_msgs/srv/SaveMap.srv)
    *   Comment: change type of filename from std_msgs/String to string
    *   Comment: Does not have a return value
    *   **Suggested Action**: consider updating type and adding return value
    *   **Followup Issue** https://github.com/ros-planning/navigation_msgs/issues/17
*   [https://github.com/ros-planning/navigation_msgs/blob/jade-devel/map_msgs/srv/SetMapProjections.srv](https://github.com/ros-planning/navigation_msgs/blob/jade-devel/map_msgs/srv/SetMapProjections.srv)
    *   Comment: should be named GetMapProjections ?
    *   **Suggested Action:** Review usage and documentation for a potential change in name.
    *   **Followup Issue** https://github.com/ros-planning/navigation_msgs/issues/16

#### move_base_actions

Comment: Remove- move base msgs are unused in ROS2 Navigation

**Suggested Action**: Check for usage and if none, do not release into Foxy

**Followup Issue** https://github.com/ros-planning/navigation_msgs/pull/15



*   [https://github.com/ros-planning/navigation_msgs/blob/jade-devel/move_base_msgs/action/MoveBase.action](https://github.com/ros-planning/navigation_msgs/blob/jade-devel/move_base_msgs/action/MoveBase.action)
