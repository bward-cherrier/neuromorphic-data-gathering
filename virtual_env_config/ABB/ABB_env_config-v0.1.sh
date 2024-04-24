#!/bin/bash

######################## Note! #############################
# Author: XXC		Data: 2024/04/24 ###################
# https://github.com/xxcStella/Dobot-Synsense/tree/main
############################################################

# User info input
echo "Current time: `date`"
echo "Please input your virtual environment name (e.g. ABB_env):"
read ENV_NAME
echo "Please input your Python interpreter version (e.g. 3.8):"
read PYTHON_VERSION
echo "Please input your common_robot_interface (CRI) folder path (e.g. "/home/user/common_robot_interface"):"
read CRI_PATH
echo "Please input your tactile-core-neuro folder path (e.g. "/home/user/tactile-core-neuro"):"
read TCN_PATH
echo "Please input your data path (e.g. "/home/user/Data"):"
read DATA_PATH

echo -e "\n"
printf "%-36s %-25s\n" "Virtual environment name:" 			$ENV_NAME
printf "%-36s %-25s\n" "Python version:"				$PYTHON_VERSION
printf "%-36s %-25s\n" "CRI folder path:"				$CRI_PATH
printf "%-36s %-25s\n" "tactile-core-neuro folder path:"		$TCN_PATH
printf "%-36s %-25s\n" "data storage path:"				$DATA_PATH

echo -e "\nShall we continue? (y/n)"
read judge
if [ $judge = 'y' ]
then
    echo -e "Continue processing.\n"
elif [ $judeg = 'n' ]
then
    echo "Stop the process."
    exit
else
    echo "Wrong input."
    exit
fi

# Check whether Conda cmd is able to use
echo -e "\n"
echo "##################################"
echo "Basic environment configuration"
echo "##################################"

if ! command -v conda &> /dev/null
then
    echo "Conda could not be found, please check your Anaconda installation!"
    exit
fi

# Create virtual env
echo "Creating a virtual environment named ${ENV_NAME}, with Python ${PYTHON_VERSION}."
conda create -n ${ENV_NAME} python=${PYTHON_VERSION} -y

# Activate virtual env
echo "Activating virtual environment."
source activate $ENV_NAME

# Install dependent libs
echo "Installing dependent libraries."
pip install numpy opencv-python PyQt5 Pyro5

# Enter CRI folder and install it
echo -e "\n"
echo "#############################################"
echo "#### Common-robot-interface Installation ####"
echo "#############################################"

echo "Changing directory to CRI folder: $CRI_PATH"
cd $CRI_PATH
echo "Installing CRI in current virual environment"
pip install -e . --user

# Enter TCN folder and install it
echo -e "\n"
echo "#############################################"
echo "##### tactile-core-neuro Configuration ######"
echo "#############################################"

config_file="$HOME/.bashrc"
echo "export PYTHONPATH=\"$TCN_PATH/python:\$PYTHONPATH\"" >> $config_file
echo "export DATAPATH=\"$DATA_PATH\"" >> $config_file

echo "Added $TCN_PATH and the Core folder to PYTHONPATH in the $ENV_NAME environment."

# INstall dv-processing
echo -e "\n"
echo "#############################################"
echo "##### tactile-core-neuro Configuration ######"
echo "#############################################"

pip install dv-processing dv

echo -e "\nABB environment is successfully set up!"



























