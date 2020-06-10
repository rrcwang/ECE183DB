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
  * scipy
  * numba

## Installation:
1. Clone and download this repo.
2. Download and run Webots. 
3. If on a **Windows OS**, download the Microsoft C++ Build Tools. It is not necessary to install any of the other packages available in the installer.
4. Download and install Python 3
5. Install required python packages. In a command console, run in the root directory
```
python -m pip install requirements.txt
```

## Instructions to run chasing demo:
1. Run setup_experiment.py
    * Specify the frisbee's initial launch velocity, angle, and spin rate
    * Each input should be a number
2. Open `183DASimul.wbt` under .\183DASimul\worlds. If everything is working, you should see something like this:
  ![Webots](https://github.com/rrcwang/ECE183DB-Project/blob/master/photos/webots.png)

The simulation will play automatically upon loading. To replay the simulation, pause and rewind the simulation to reset it to its initial state. These controls can be found at the top of the simulation window. You have the option to move and resize the 3 square display windows in the simulation window. The yellow, pink, and cyan windows are the rangefinder output, camera output, and visual respectively. 

# Explaining the visual:
![visualizer](https://github.com/rrcwang/ECE183DB-Project/blob/master/photos/visualizer.png)

The visual shows the Pursuit Algorithm as well as the progression of the path as it is output by the Kalman Filter. 
* The __white points__ represent the path that the Path Predictor thinks the frisbee will take based on incoming sensor measurements. 
* The __green point__ is the 2D position estimation of the car itself. 
* The __cyan point__ is the 2D position estimation of the frisbee. The __red circle__ is centered on the car and is the lookahead distance. 
* The point of the path which intersects this circle is the lookahead point. The path extends past the destination point so that the car will know to drive towards it when it is close. Note that this visual is essentially what the car “sees” as the frisbee state estimate and path are generated from sensor data. 
* The turning radius and speed come from the Pure Pursuit Algorithm and a PID controller designed to try to stay under the frisbee.

3D model of Ultrastar frisbee by Isaac Dunbar. https://grabcad.com/library/blue-ultrastar-flying-disc-frisbee-1
