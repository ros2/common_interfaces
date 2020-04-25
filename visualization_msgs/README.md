# visualization_msgs

This package provides messages for visualizing 3D information in ROS GUI programs, particularly RViz.

These messages were ported from ROS 1 and for now the [visualization_msgs wiki](http://wiki.ros.org/visualization_msgs) is still a good place for information about these messages and how they are used.

For more information about ROS 2 interfaces, see [index.ros2.org](https://index.ros.org/doc/ros2/Concepts/About-ROS-Interfaces/)

## Messages (.msg)
* [ImageMarker](ImageMarker.msg): A marker to overlay on displayed images.
* [InteractiveMarker](InteractiveMarker.msg): A user interaction marker for manipulating objects in 3-dimensional space in GUI programs, like RViz.
* [InteractiveMarkerControl](InteractiveMarkerControl.msg): Represents a control that is to be displayed together with an interactive marker.
* [InteractiveMarkerFeedback](InteractiveMarkerFeedback.msg): Feedback message sent back from the GUI, e.g. when the status of an interactive marker was modified by the user.
* [InteractiveMarkerInit](InteractiveMarkerInit.msg): Used for sending initial interactive marker descriptions.
* [InteractiveMarkerPose](InteractiveMarkerPose.msg): The pose of the interactive marker.
* [InteractiveMarkerUpdate](InteractiveMarkerUpdate.msg): The top-level message for sending data from the interactive marker server to the client (i.e. rviz).
* [Marker](Marker.msg): A non-interactive marker for displaying annotations in 3-dimensional space.
* [MarkerArray](MarkerArray.msg): An array of markers.
* [MenuEntry](MenuEntry.msg): Used to describe the menu/submenu/subsubmenu/etc tree.

## Services (.srv)
* [GetInteractiveMarkers.srv](GetInteractiveMarkers.srv): Get the currently available interactive markers.

## Quality Declaration
This package claims to be in the **Quality Level 4** category, see the [Quality Declaration](QUALITY_DECLARATION.md) for more details.
