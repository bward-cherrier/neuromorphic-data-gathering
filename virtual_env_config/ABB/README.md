Author: XXC

Data: 2024/04/24

Note: Always use the latest version!

This repository contains a shell script, which is used to automatically configure ABB environment.

Procedures:

1, Download CRI and tactile-core-neuro files. (You can find them in this Github repository or use links below)

  CRI link: https://github.com/dexterousrobot/common_robot_interface
  
  TCN link: https://bitbucket.org/bw14452/tactile-core-neuro/src/master/

2, Download this shell scipt file.

   Set the mode of this bash file to executive: 
   
    "chmod +x PATH_OF_THE_FILE"
    
3, Create an empty folder to store all your future data.

4, cd to your bash file folder:

    e.g. cd /home/user/Robots-Sensors/virtual_env_config/

4, Run the bash file:

    "PATH_OF_THE_FILE/ABB_env_config-v0.1.sh"

Notes:

1, Linux system is required.

2, Anaconda software is required (better add it to the system PATH when installing).
