^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package carl_safety
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.6 (2015-04-17)
------------------
* Added error feedback messages
* travis fix
* Contributors: David Kent, Russell Toris

0.0.5 (2015-04-08)
------------------
* carl safety now pipes output to dev null
* Contributors: Russell Toris

0.0.4 (2015-02-17)
------------------
* Adjusted finger current thresholds
* Fixed output message for finger joints on arm safety
* Adjustments for finger safety threshold
* Merge branch 'develop' of https://github.com/WPI-RAIL/carl_safety into develop
* Arm safety threshold tuning
* Contributors: David Kent

0.0.3 (2015-02-06)
------------------
* message generation dependency
* Split launch file into basic safety nodes that should always be on and nodes that only affect safety when using an external interface
* launch file for tipping safety
* Launch file updated with tipping safety
* Tuned nav safety thresholds and added tipping safety to stop the robot when tipping is detected
* Changed manual and auto nav safety to allow movement when the arm is within CARL's navigation footprint
* Removed some debugging statements
* Manual base movement override if arm is not retracted
* Contributors: David Kent

0.0.2 (2014-12-04)
------------------
* Changed int param to bool
* Parameter, topic, and launch file cleanup for consistency
* Merge pull request `#2 <https://github.com/WPI-RAIL/carl_safety/issues/2>`_ from bhetherman/develop
  changes to make teleop safety and arm noise able to toggle on and off with launch paramater
* changes to make teleop safety and arm noise able to toggle on and off with launch paramater
* Update .travis.yml
* Update package.xml
* merged
* Added dependency on wpi_jaco_msgs for safe nav
* Implemented /move_base_safe to retract the arm before navigation
* Contributors: Brian Hetherman, David Kent, Russell Toris

0.0.1 (2014-09-05)
------------------
* cleanup for release
* adjusted safety override near computer desks
* Added discouragement for CARL attempting to crash into our computers, remote run stop should now work for autonomous nav as well
* slowed down spin rate
* created launch file, added dependency on robot_pose_publisher for launch
* implemented boundary for manual nav
* more debugging
* debugging
* initial commit
* Contributors: Russell Toris, dekent
