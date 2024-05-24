#!/bin/bash

######################## Note! #############################
# Author: XXC		Data: 2024/04/26 ###################
############################################################

# User editable items
echo "Current time: `date`"
echo "Please input your virtual environment name (or press Enter to use default name: Specktac):"
read ENV_NAME
ENV_NAME=${ENV_NAME:-'Specktac'}
echo "Please input your Python interpreter version (or press Enter to use default version: 3.8):"
read PYTHON_VERSION
PYTHON_VERSION=${PYTHON_VERSION:-'3.8'}

cd ../../
current_dir=$(pwd)
CRI_PATH="$current_dir/source_code/common_robot_interface"
SINABS_PATH="$current_dir/source_code/sinabs"

echo -e "\n"
printf "%-28s %-25s\n" "Virtual environment name:" 	$ENV_NAME
printf "%-28s %-25s\n" "Python version:"		$PYTHON_VERSION
printf "%-28s %-25s\n" "CRI folder path:"		$CRI_PATH
printf "%-28s %-25s\n" "Sinabs folder path:"		$SINABS_PATH

echo "Shall we continue? [y/n]"
read judge
if [ $judge = 'y' ]
then
   echo -e "Continue processing\n"
elif [ $judge = 'n' ]
then
   echo "Stop the process"
   exit
else
   echo "Wrong input!"
   exit
fi


# 检查conda环境是否可用
echo -e "\n"
echo "##################################"
echo "Baisc environment configuration"
echo "##################################"

if ! command -v conda &> /dev/null
then
    echo "conda could not be found, please check your Anaconda installation!"
    exit
fi

# 创建虚拟环境
echo "Creating a virtual environment named ${ENV_NAMW}, with Python ${PYTHON_VERSION}."
conda create --name $ENV_NAME python=$PYTHON_VERSION -y

# 激活虚拟环境
echo "Activating the virtual environment"
source activate $ENV_NAME

# 进入CRI目录并安装CRI
echo -e "\n"
echo "#############################################"
echo "#### Common-robot-interface Installation ####"
echo "#############################################"

echo "Changing directory to CRI folder: $CRI_PATH"
cd $CRI_PATH
echo "Installing CRI in current virual environment"
pip install -e .

# Install Samna
echo -e "\n"
echo "#############################################"
echo "############ Samna Installation #############"
echo "#############################################"

echo "Installing dependent libs"
echo "Creating udev rules file for USB devices"
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="04b4", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="152a", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="337d", MODE="0666"' | sudo tee /etc/udev/rules.d/60-synsense.rules > /dev/null

echo "Reloading udev rules"
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Installing required system-level development packages"
sudo apt-get update
sudo apt-get install -y mesa-common-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libglu1-mesa-dev libxcb-dri2-0-dev libxcb-dri3-dev libxcb-present-dev libxcb-sync-dev libx11-xcb-dev libxcb-glx0-dev

echo "Installing and importing samna to temp python file to update it"
pip install samna
echo "import samna" > temp_script.py
python temp_script.py
rm temp_script.py

# Install Sinabs
echo -e "\n"
echo "#############################################"
echo "############ Installing Sinabs ##############"
echo "#############################################"

cd $SINABS_PATH
pip install -e .

# Install Sinabs-dynapcnn
# This part is no longer required!! This lib is merged into sinabs!!
# echo "#############################################"
# echo "######## Installing Sinabs-dynapcnn #########"
# echo "#############################################"
# pip install sinabs-dynapcnn

echo -e "\nSpecktac environment is successfully configured!"



