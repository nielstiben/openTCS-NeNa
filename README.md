# NeNa ROS 2 Vehicle Driver For OpenTCS.

OpenTCS and ROS2 are both open-source software packages. OpenTCS can be used as a fleet manager to autonomously manage self-driving vehicles. ROS2 is a widely used software package that takes care of common functionality that many AGVs share, such as support for sensors, cameras and SLAM. 

 At the moment of writing, there are no (open-source) software packages available to manage ROS2 robots from the OpenTCS Fleet Manager. The development of such a vehicle driver is part of my Bachelor Thesis.

The following software and software libraries is used for developing the driver:
* ROS 2 - dashing
* [ROS2-java (rcljava)](https://github.com/ros2-java/ros2_java)
* [OpenTCS](https://www.opentcs.org/en/index.html)

## Features
**NOTE:** *The OpenTCS-ROS2 driver has only been tested on Ubuntu 18.04 in combination with the [Turtlebot 3](http://emanual.robotis.com/docs/en/platform/turtlebot3/ros2_setup/)*.

All the native OpenTCS features are implemented in this driver, which means that the driver is ready to be used.

General features:
* Support for OpenTCS Transport Orders.
* Support for OpenTCS Operations.
* Properly handle ROS2 AGV navigation failures, such an unreachable destination.
* Track the live position of a ROS2 AGV for showing in the OpenTCS Plant Overview.
* Live orientation of the ROS2 AGV for showing in the OpenTCS Plant Overview.
* Adjustable ROS2 namespaces, which allows usage for multiple ROS2 AGVs simultaneously.
* Dynamic ROS2 domain IDs.
* Plant Model scaling, allowing better representation of tiny/huge plant maps.


Control panel features:
* Dispatch the ROS2 AGV to a user-defined coordinate in the control center pane.
* Dispatch the ROS2 AGV to an OpenTCS Plant Model point.
* Set the initial position of a ROS2 AGV.
* Show a continuously updated ROS2 navigation status in the control center panel.
* Show the connection status in the control center panel.

Todo:

* Implement smooth navigation: drive to a destination without stopping at passing points along the route.
* Get a percentage of how much a current navigation goal has been completed.
* Show a battery percentage of the vehicle in the OpenTCS Plant Overview.

## Demo
[![Alt text](openTCS-NeNa-Documentation/src/docs/img/youtube_embed.png)](https://youtu.be/x_Bjo7l0uc4)

## User Guide
Here's [a guide about how to setup and use the ROS 2 Driver](openTCS-NeNa-Documentation/src/docs/user_guide/user_guide.adoc) ( [asciidoc](openTCS-NeNa-Documentation/src/docs/user_guide/user_guide.adoc) | [pdf](openTCS-NeNa-Documentation/src/docs/user_guide/user_guide.pdf) | [odt](openTCS-NeNa-Documentation/src/docs/user_guide/user_guide.odt) ).

## Developer Guide
Here's [documentation about the development of the ROS 2 Driver](openTCS-NeNa-Documentation/src/docs/developers_guide/developers_guide.adoc) ( [asciidoc](openTCS-NeNa-Documentation/src/docs/developers_guide/developers_guide.adoc) | [pdf](openTCS-NeNa-Documentation/src/docs/developers_guide/developers_guide.pdf) | [odt](openTCS-NeNa-Documentation/src/docs/developers_guide/developers_guide.odt) ).

## Other Documents
~~Research ROS 2 communication (pdf).~~ Will be uploaded soon.

~~Bachelor Thesis: Fleet Management And Annotation Mapping For NeNa Robots (pdf).~~ Will be uploaded soon.