# Readme

Author: XXC

Contributor: Ben

Data: 2024/04/24

Note: Always use the latest version!

This folder contains code for robots, sensors and environment configuration.

--------------------------------------------------------
## Acknowledgement

You can find source code from these links (For stablization purpose, try to use libs from source_code folder):

1. CRI: https://github.com/dexterousrobot/common_robot_interface

2. tactile-core-neuro: https://bitbucket.org/bw14452/tactile-core-neuro/src/master/

3. sinabs: https://sinabs.readthedocs.io/en/v1.2.10/getting_started/install.html

--------------------------------------------------------

## Composition

* source_code: source code for controlling robots and sensors

    -  ABB-setup

        1. common_robot_interface: Used to control ABB and dobot.

        2. tactile-core-neuro: Used to collect NeuroTac data.

    - sinabs

        1. Stable sinabs version.

* setup_shell_scripts (shell scripts to automatically set up robot and sensor env):

    - ABB_NeuroTac: To configure ABB and NeuroTac env.

    - dobot_synsense: To configure dobot and synsense env.

--------------------------------------------------------

## Guidence

1. Download or Git clone this repo to your desired path (e.g. /home/user/Robots-Sensors).

2. cd to your goal shell folder

	e.g. cd /home/user/Robots-Sensors/setup_shell_scripts/ABB_NeuroTac

3. Read "Readme.md" file in the shell scripts folder.
