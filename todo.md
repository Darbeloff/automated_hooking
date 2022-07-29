# TODO

- Upload the env_loader.sh script to github somewhere
- Figure out required odrive version and document it
- fix gantry velocity control
- combine controls and utils files (make python package?)
- winch_node controls different winches in different modes
- parametrize apriltag scanner from local files
- refactor controls package to allow lambda input
- auto-generate apriltag description based on target_description.urdf

- ~~make or modify odrive class so controlling all odrives is easier~~
- ~~OdriveClass gets amp-control mode, is placed in Util file~~
- ~~give gantry initial location guess~~
- ~~figure out description publisher; make gantry_description and target_description xacros, import them into one system_description~~
- ~~finish gantry_description~~
- ~~finish target_description~~
- ~~redo tf of the april_tag detector? or does the tf_buffer already take care of this?~~
- ~~determine why it fails sometimes~~
- ~~Fix postioning code ~~