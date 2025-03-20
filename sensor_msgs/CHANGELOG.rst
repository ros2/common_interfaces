^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sensor_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

5.3.6 (2025-03-20)
------------------

5.3.5 (2024-04-24)
------------------

5.3.4 (2024-04-16)
------------------

5.3.3 (2024-04-10)
------------------

5.3.2 (2024-04-10)
------------------
* Clarify the license. (`#241 <https://github.com/ros2/common_interfaces/issues/241>`_)
  In particular, every package in this repository is Apache 2.0
  licensed except for sensor_msgs_py.  So move the CONTRIBUTING.md
  and LICENSE files down into the individual packages, and
  make sure that sensor_msgs_py has the correct CONTRIBUTING.md
  file (it already had the correct LICENSE file).
* [J-Turtle] Fix uninitialized values in NavSatFix and add missing NavSatStatus UNKNOWN (`#220 <https://github.com/ros2/common_interfaces/issues/220>`_)
  * Fix unitialized values in NavSatFix and add missing UNKNOWN
  * Fixes `#196 <https://github.com/ros2/common_interfaces/issues/196>`_
  * Fix default initialization instead of constants
  * Define SERVICE_UNKNOWN
  Co-authored-by: Tully Foote <tully.foote@gmail.com>
  Co-authored-by: Martin Pecka <peci1@seznam.cz>
* Contributors: Chris Lalancette, Ryan

5.3.1 (2024-03-28)
------------------
* Use target qualifier for checking the cpp typesupport exists (`#238 <https://github.com/ros2/common_interfaces/issues/238>`_)
* Contributors: Ryan

5.3.0 (2024-01-24)
------------------

5.2.2 (2023-12-26)
------------------
* sensor_msgs/CompressedImage: updated description of format field (`#231 <https://github.com/ros2/common_interfaces/issues/231>`_)
* Contributors: Kenji Brameld

5.2.1 (2023-11-06)
------------------
* Return true for isColor if format is YUYV or UYUV (`#229 <https://github.com/ros2/common_interfaces/issues/229>`_)
* Contributors: Kenji Brameld

5.2.0 (2023-06-07)
------------------

5.1.0 (2023-04-27)
------------------

5.0.0 (2023-04-11)
------------------
* update YUV format codes and documentation (`#214 <https://github.com/ros2/common_interfaces/issues/214>`_)
* sensor_msgs/Range lacks variance field (`#181 <https://github.com/ros2/common_interfaces/issues/181>`_)
* Contributors: Christian Rauch, El Jawad Alaa

4.7.0 (2023-02-13)
------------------
* Update common_interfaces to C++17. (`#215 <https://github.com/ros2/common_interfaces/issues/215>`_)
* [rolling] Update maintainers - 2022-11-07 (`#210 <https://github.com/ros2/common_interfaces/issues/210>`_)
* Replaced non-ASCII dash symbol with ASCII dash (`#208 <https://github.com/ros2/common_interfaces/issues/208>`_)
* Contributors: Audrow Nash, Chris Lalancette, Ivan Zatevakhin

4.6.1 (2022-11-02)
------------------
* Add NV21 and NV24 to colour formats (`#205 <https://github.com/ros2/common_interfaces/issues/205>`_)
* Update BatteryState.msg (`#206 <https://github.com/ros2/common_interfaces/issues/206>`_)
* Contributors: Borong Yuan, Geoffrey Biggs

4.6.0 (2022-09-13)
------------------
* use regex for matching cv types (`#202 <https://github.com/ros2/common_interfaces/issues/202>`_)
* Fix outdated file path for image_encodings (`#200 <https://github.com/ros2/common_interfaces/issues/200>`_)
* Use uint32_t for pointcloud2 resize method (`#195 <https://github.com/ros2/common_interfaces/issues/195>`_)
* Retain width and height after resize for master (`#193 <https://github.com/ros2/common_interfaces/issues/193>`_)
* Contributors: Kenji Brameld, Tianyu Li

4.5.0 (2022-05-19)
------------------

4.4.0 (2022-04-29)
------------------

4.3.0 (2022-04-29)
------------------

4.2.1 (2022-03-31)
------------------
* Move the find_package statements for BUILD_TESTING  (`#186 <https://github.com/ros2/common_interfaces/issues/186>`_)
* Contributors: Michael Jeronimo

4.2.0 (2022-03-30)
------------------

4.1.1 (2022-03-26)
------------------
* Feedback on conditional sensor_msgs_library target (`#1 <https://github.com/ros2/common_interfaces/issues/1>`_) (`#183 <https://github.com/ros2/common_interfaces/issues/183>`_)
* [Fix] Fix image_encodings.hpp's URL in README (`#184 <https://github.com/ros2/common_interfaces/issues/184>`_)
* [Fix] Fix fill_image.hpp's URL in README (`#182 <https://github.com/ros2/common_interfaces/issues/182>`_)
* Add sensor_msgs_library target and install headers to include/${PROJECT_NAME} (`#178 <https://github.com/ros2/common_interfaces/issues/178>`_)
* Contributors: Homalozoa X, Pablo Garrido, Shane Loretz

4.1.0 (2022-03-01)
------------------
* Interface packages should fully <depend> on the interface packages that they depend on (`#173 <https://github.com/ros2/common_interfaces/issues/173>`_)
* Add YUV420 and YUV444 to image encodings (`#172 <https://github.com/ros2/common_interfaces/issues/172>`_)
* Contributors: Grey, Hemal Shah

4.0.0 (2021-12-14)
------------------
* Cleanup mislabeled BSD license (`#83 <https://github.com/ros2/common_interfaces/issues/83>`_)
* Update maintainers to Geoffrey Biggs and Tully Foote (`#163 <https://github.com/ros2/common_interfaces/issues/163>`_)
* Fix rosdoc2 warnings in sensor_msgs. (`#162 <https://github.com/ros2/common_interfaces/issues/162>`_)
* Add equidistant distortion model (`#160 <https://github.com/ros2/common_interfaces/issues/160>`_)
* Contributors: Audrow Nash, Chris Lalancette, Martin Günther, Tully Foote

3.0.0 (2021-08-24)
------------------

2.3.0 (2021-08-11)
------------------
* Use rosidl_get_typesupport_target() (`#156 <https://github.com/ros2/common_interfaces/issues/156>`_)
* Update CompressedImage documentation: add 'tiff' as a supported format (`#154 <https://github.com/ros2/common_interfaces/issues/154>`_)
* Contributors: Ivan Santiago Paunovic, Shane Loretz

2.2.3 (2021-04-27)
------------------

2.2.2 (2021-04-06)
------------------
* Change index.ros.org -> docs.ros.org. (`#149 <https://github.com/ros2/common_interfaces/issues/149>`_)
* updating quality declaration links (re: `ros2/docs.ros2.org#52 <https://github.com/ros2/docs.ros2.org/issues/52>`_) (`#145 <https://github.com/ros2/common_interfaces/issues/145>`_)
* Contributors: Chris Lalancette, shonigmann

2.2.1 (2021-01-25)
------------------
* Fix PointCloud2Iterator namespacing in docs (`#139 <https://github.com/ros2/common_interfaces/issues/139>`_)
* Contributors: Andre Nguyen

2.2.0 (2020-12-10)
------------------
* Add coverage/performance to qd for sensor_msgs (`#137 <https://github.com/ros2/common_interfaces/issues/137>`_)
* Update QDs to QL 1 (`#135 <https://github.com/ros2/common_interfaces/issues/135>`_)
* Update package maintainers. (`#132 <https://github.com/ros2/common_interfaces/issues/132>`_)
* Updated Quality Level to 2 (`#131 <https://github.com/ros2/common_interfaces/issues/131>`_)
* Contributors: Alejandro Hernández Cordero, Michel Hidalgo, Stephen Brawner

2.1.0 (2020-07-21)
------------------

2.0.2 (2020-07-21)
------------------
* Missing cstring header for memcpy in fill_image.hpp (`#126 <https://github.com/ros2/common_interfaces/issues/126>`_)
* Update Quality levels to level 3 (`#124 <https://github.com/ros2/common_interfaces/issues/124>`_)
* Add Security Vulnerability Policy pointing to REP-2006. (`#120 <https://github.com/ros2/common_interfaces/issues/120>`_)
* Contributors: Chris Lalancette, Jose Luis Rivero, brawner

2.0.1 (2020-05-26)
------------------
* QD Update Version Stability to stable version (`#121 <https://github.com/ros2/common_interfaces/issues/121>`_)
* Contributors: Alejandro Hernández Cordero

1.0.0 (2020-05-20)
------------------
* Improve clarification of MultiDOFJointState (`#114 <https://github.com/ros2/common_interfaces/issues/114>`_)
* Comment on common PointField names (`#112 <https://github.com/ros2/common_interfaces/issues/112>`_)
* Fix sensor_msgs README (`#111 <https://github.com/ros2/common_interfaces/issues/111>`_)
* Add current-level quality declarations (`#109 <https://github.com/ros2/common_interfaces/issues/109>`_)
* Contributors: Tully Foote, brawner
