Author: XXC

Data: 2024/04/23

Note: Always use the latest version!

This repository contains a shell script, which is used to automatically configure dobot and synsense environment.

Procedures:

1, Download or Git clone this repo to your desired path (e.g. /home/user/Robots-Sensors).

2, cd to your goal shell folder

    e.g. cd /home/user/Robots-Sensors/setup_shell_scripts/ABB_NeuroTac

   Set the mode of this bash file to executive: 
   
    chmod +x PATH_OF_THE_FILE

3, Run the bash file:

    PATH_OF_THE_FILE/dobot_env_config-vx.x.sh
    
4, Change the TCP address and select the right port.

Notes:

1, This process can take a long time if you don't have CUDA implemented before!

2, It was only tested on computer with a GPU. No-GPU-computers may generate errors.

3, Linux system is required.

4, Anaconda software is required (better add it to the system PATH when installing).
