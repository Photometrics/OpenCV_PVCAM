# Calling PVCAM within OpenCV

This repository contains a script that will install OpenCV on Linux, it dependencies, and modify it so that it can be used with Photometrics camera's

## Getting Started
Follow the bellow instructions to get OpenCV installed and able to communicate with Photometrics camera's

### Prerequisites
* You must already have the PVCAM driver installed which can be can be downloaded [here](https://www.photometrics.com/support/software/#software).
* You will also need the PVCAM sdk installed which is available upon request

### Installing
After you have cloned the repository, navigate to the folder that contains the opencv_install.sh script in a terminal and run 
```sudo ./opencv_install.sh``` 

During the install process for cuda it will prompt the user for input. When it asks if you want to install NVIDIA Accelerated Graphics Drivers choose no. Otherwise choose yes or the default. 

### How to use OpenCV with Photometrics Camera's
In the sample folder there is an example of how you can use OpenCV to communicate with Photometrics Camera's

You can compile and run this sample by navigating to the directory with the source code and the CMakeLists file and running the following commands.

```mkdir build; cd build```

```cmake ..; make```

```./opencv_example```

###Notes
This code is provided as-is, with no warranty, service, or support. It demonstrates how PVCAM library calls can be accessed from within OpenCV.  It has only been tested on Ubuntu 16.04 on a Dell Precision T3610
