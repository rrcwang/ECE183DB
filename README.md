# ECE183DB
Created by Richard W., Timothy L., and Christian Yu for the UCLA ECE183DB Capstone Design Project

## Requirements:
* Webots:                           https://www.cyberbotics.com/#download
* If on **Windows**:
    * Microsoft C++ Build Tools:    https://visualstudio.microsoft.com/visual-cpp-build-tools/
* Python 3:                         https://www.python.org/downloads/
* Python libraries:
  * numpy
  * numpy-quaternion

## Installation:
0. Clone and download this repo.
1. Download and run Webots. If everything is working, you should see something like this:

![alt text][logo]  
[logo]: 1 "Logo Title Text 2"  

2. If on a **Windows OS**, download the Microsoft C++ Build Tools:
  * It is not necessary to install any of the other packages available in the installer.
3. Download and install Python 3
4. Install required packages:
  * In a command console in the root directory
```
python -m pip install requirements.txt
```

## Instructions to run chasing demo:
1. Run setup_experiment.py
    * Specify the frisbee's initial launch velocity, angle, and spin rate
    * Each input should be a number

2. Open `183DASimul.wbt` under .\183DASimul\worlds
3. Hit the play button on the 



3D model of Ultrastar frisbee by Isaac Dunbar. https://grabcad.com/library/blue-ultrastar-flying-disc-frisbee-1