#!/bin/bash

######################## Note! #############################
# Author: XXC		Data: 2024/04/26 ###################
############################################################


# User info input
echo "Current time: `date`"
echo "Please input your virtual environment name (or press Enter to use default name: neurotac):"
read ENV_NAME
ENV_NAME=${ENV_NAME:-'neurotac'}
echo "Please input your Python interpreter version (or press Enter to use default version: 3.8):"
read PYTHON_VERSION
PYTHON_VERSION=${PYTHON_VERSION:-'3.8'}

cd ../../
current_dir=$(pwd)
CRI_PATH="$current_dir/source_code/common_robot_interface"
TCN_PATH="$current_dir/source_code/tactile-core-neuro"

default_data_dir=$HOME/Data
echo "Please input your data path (or press Enter to use the default path: $default_data_dir):"
read DATA_PATH
DATA_PATH=${DATA_PATH:-$default_data_dir}

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
pip install numpy opencv-python PyQt5 Pyro5 h5py

# Enter CRI folder and install it
echo -e "\n"
echo "#############################################"
echo "#### Common-robot-interface Installation ####"
echo "#############################################"

echo "Changing directory to CRI folder: $CRI_PATH"
cd $CRI_PATH
echo "$CRI_PATH"
echo "Installing CRI in current virual environment"
pip install -e .

# Enter TCN folder and install it
echo -e "\n"
echo "#############################################"
echo "##### tactile-core-neuro Configuration ######"
echo "#############################################"

config_file="$HOME/.bashrc"
echo "export PYTHONPATH=\"$TCN_PATH/python:\$PYTHONPATH\"" >> $config_file
echo "export DATAPATH=\"$DATA_PATH\"" >> $config_file
source $config_file

echo "Added $TCN_PATH and the Core folder to PYTHONPATH in the $ENV_NAME environment."

# INstall dv-processing
echo -e "\n"
echo "#############################################"
echo "##### dv-processing installation ######"
echo "#############################################"

pip install dv-processing dv

echo -e "\nNeurotac environment is successfully set up!"



























