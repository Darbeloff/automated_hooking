# Code Structure

## Controllers and Nodes


## Utility Files
### Coord

This file defines a Coord class to represent translations/orientations in the homogeneous coordinate format. It can be constructed from a homogeneous coordinate matrix directly, from translation and orientation vectors, from a Transform ROS message, or from any ROS message with a translation/position and rotation/orientation defined.

The built-in addition and multiplication methods will append the second transform to the first (by matrix multiplying them).

The built-in subtraction and division methods will invert the second transform, then append it to the first.

The built-in _abs_ method gets the norm of the translation of the Coord, as if it were a vector.

It also contains handy methods to turn the Coord object back into an orientation, translation, Pose, or Transform format


### ODrive

This file contains an ODrive class (built heavily of previous work) that does most of the annoying interfacing with the python odrive package. Notable differences from different versions of this class include the ability to add more than one ODrive board to a single instance of the ODrive object. One object can then get sensor data from all of the attached motors and write commands.