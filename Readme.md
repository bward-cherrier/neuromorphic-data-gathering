# Readme

Author: XXC

Contributor: Ben

Date: 2024/04/24

Note: Always use the latest version!

This repository contains code for data gathering with neuromorphic tactile sensors (Neurotac, Specktac) attached to robot arms (ABB IRB120, UR5, DOBOT MG400)

## Guidance

1. Download or Git clone this repo to your desired path (e.g. /home/user/Robots-Sensors).

2. cd to your goal setup folder based on the sensor you are using.

	e.g. cd /home/user/Robots-Sensors/setup/NeuroTac

3. Read the "README.md" file in the setup folder for instructions on how to run the relevant shell script.

--------------------------------------------------------

## Composition

* source_code: source code for controlling robots and sensors

        1. common_robot_interface: Used to control ABB, UR5 and dobot.

        2. tactile-core-neuro: Used to collect Neurotac data.

        3. sinabs: Used to collect Specktac data.

* setup (shell scripts to automatically set up robot and sensor env):

    - Neurotac: To configure a conda environment for the  NeuroTac sensor

    - Specktac: To configure a conda environment for the Specktac sensor

--------------------------------------------------------


## Acknowledgement

Source code was obtained from the following links (For stablization purpose, use libs already present in the source_code folder):

1. CRI: https://github.com/dexterousrobot/common_robot_interface

2. tactile-core-neuro: https://bitbucket.org/bw14452/tactile-core-neuro/src/master/

3. sinabs: https://sinabs.readthedocs.io/en/v1.2.10/getting_started/install.html


--------------------------------------------------------
