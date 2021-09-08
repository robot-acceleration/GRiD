# GRiD

A GPU accelerated library for computing rigid body dynamics with analytical gradients.

GRiD wraps our [URDFParser](https://github.com/robot-acceleration/URDFParser), [GRiDCodeGenerator](https://github.com/robot-acceleration/GRiDCodeGenerator), and [RBDReference](https://github.com/robot-acceleration/RBDReference) packages. Using its scripts, users can easily generate and test optimized rigid body dynamics CUDA C++ code for their URDF files.

**This package contains submodules make sure to run ```git submodule update --init --recursive```** after cloning!

## Dependencies:
In order to support the wrapped packages there are 4 required external packages ```beautifulsoup4, lxml, numpy, sympy``` which can be automatically installed by running:
```shell
pip3 install -r requirements.txt
```
Running the CUDA C++ code output by the GRiDCodegenerator also requires CUDA to be installed on your system. Please see below for CUDA installation instructions.

## Usage and API:
+ To generate the ```grid.cuh``` header file please run: ```generateGRiD.py PATH_TO_URDF (-D)``` where ```-D``` indicates full debug mode which will include print statements after ever step of ever algorithm
+ To test the python refactored algorithms against our reference implmentations please run ```testGRiDRefactorings.py PATH_TO_URDF (-D)``` where ```-D``` prints extra debug values as compared to just the comparisons
+ To print and compare GRiD to reference values please do the following steps: 
  1) Print the reference values by running ```printReferenceValues.py PATH_TO_URDF (-D)``` where ```-D``` prints the full debug reference values from the refactorings 
  2) Run ```printGrid.py PATH_TO_URDF (-D)``` to compile, run, and print the same values from CUDA C++

## CUDA Instalation Instructions
### Install dependencies
```
sudo apt-get update
sudo apt-get -y install xorg xorg-dev linux-headers-$(uname -r) apt-transport-https
```
### Download and install CUDA 
Note: for Ubuntu 20.04 see [https://developer.nvidia.com/cuda-downloads](https://developer.nvidia.com/cuda-downloads) for other distros
```
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/7fa2af80.pub
sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/ /"
sudo apt-get update
sudo apt-get -y install cuda
```
### Add the following to ```~/.bashrc```
```
export PATH="/usr/local/cuda/bin:$PATH"
export LD_LIBRARY_PATH="/usr/local/cuda/lib64:$LD_LIBRARY_PATH"
export PATH="opt/nvidia/nsight-compute/:$PATH"
```
