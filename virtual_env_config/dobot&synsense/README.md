Author: XXC

Data: 2024/04/23

Note: Always use the latest version!

This repository contains a shell script, which is used to automatically configure dobot and synsense environment.

Procedures:

1, Download CRI and Sinabs files. (You can find them in this Github repository or use links below)

  CRI link: https://github.com/dexterousrobot/common_robot_interface
  
  Sinabs link: https://github.com/synsense/sinabs

2, Download this shell scipt file.

   Set the mode of this bash file to executive: 
   
    "chmod +x PATH_OF_THE_FILE"

3, Run the bash file:

    "PATH_OF_THE_FILE/dobot_env_config-vx.x.sh"

Notes:

1, This process can take a long time if you don't have CUDA implemented before!

2, It was only tested on computer with a GPU. No-GPU-computers may generate errors.

3, Linux system is required.

4, Anaconda software is required (better add it to the system PATH when installing).
