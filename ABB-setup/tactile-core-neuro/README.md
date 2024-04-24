# BRL Neuromorphic Touch Group - Core Software Framework#

This is the central repository for the BRL Neuromorphic Touch Group core software framework.

## Getting Started ##

To get started, please see the [Getting Started](https://bitbucket.org/brltactile/tactile-core/wiki/Getting%20Started) section of the wiki.

## Contribution Guidelines ##

Please refer to the [Coding Guidelines](https://bitbucket.org/brltactile/tactile-core/wiki/Coding%20Guidelines) section of the wiki.

## Contacts ##

First, check the [FAQ](https://bitbucket.org/brltactile/tactile-core/wiki/FAQ) section of the wiki. 
Otherwise, you can contact the following members of the team:

* Ben Ward-Cherrier (bw14452@bristol.ac.uk)


## Installation ##

1) **Download and install Python:** 

- Download and install `Anaconda3-2019.10-Windows-x86_64`

- During installation process tick the box for including paths (produces red warning)

NB: if you have any previous Python installations, uninstall and delete remaining Anaconda directories to avoid conflicts

NB: I installed to `C:\Users\<uname>\Anaconda3`

2) **Create environment for this repository:**

- Navigate to installation directory: `C:\Users\<uname>\Anaconda3\condabin`

- Open command prompt in this directory (type `cmd` into address bar) then enter:

- `conda create -n tactip python=3.9 anaconda`

NB: to check it worked: `conda env list`

3) **Set paths**

- Open `control panel` and `edit the system environment variables`

- Edit user variables for `<uname>`

- Double click the `Path` variable

- Either edit existing paths (if ticked box for including paths) or create new paths:

	- `C:\Users\<uname>\Anaconda3\envs\tactip`

	- `C:\Users\<uname>\Anaconda3\envs\tactip\Library\mingw-w64\bin`

	- `C:\Users\<uname>\Anaconda3\envs\tactip\Library\usr\bin`

	- `C:\Users\<uname>\Anaconda3\envs\tactip\Library\bin`

	- `C:\Users\<uname>\Anaconda3\envs\tactip\Scripts`

NB: These must be the first 5 paths listed (use move up and down)

NB: If you want to activate an environment from the command line e.g. `active tactip` add the path: `C:\Users\<uname>\Anaconda3\condabin`

4) **Install Python libraries:**

- Open a command prompt in the tactip enviroment:

- Either use the windows menu to open `Anoconda Prompt (tactip)` or open command prompt (`cmd`) and `activate tactip`

- `pip install opencv-python==3.4.5.20`

- `pip install tensorflow==1.14` (NB if have GPU capability `pip install tensorflow-gpu==1.14`)

- `pip install keras`

- `pip install hyperopt` (NB if using this; NB needs visual studio)

- `python -m pip install git+https://gitlab.com/inivation/dv/dv-processing` (if using dv_processing without DV GUI)

NB: Check installed packages with: `pip freeze | more`

NB: Anaconda can be upgraded to latest releases with: `conda update --all`

NB: Sometimes numpy or scipy can need upgrading: `pip install numpy -U`, `pip install scipy -U`

NB: tensorflow 1.15 does not seem to work with Matlab (issue with paths)


5) **Other useful things:**

- Get copy of `amcap.exe` to access camera settings

- I disable my webcam on my PC to ensure USB camera in TacTip picked up

- Don't forget within Matlab to `Add to Path` the matlab directory and subdirectories

NB Using CRI: program editor: delete module SERVER; program editor: load module abb_server.mod; production window to main; press play

NB Using CRI: robot jogger: ABB 5000


