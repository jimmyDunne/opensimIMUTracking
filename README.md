opensimIMUTracking
============
Matlab code that was used to prototype, test, and develop the latest IMU tracking methods in OpenSim 4.1.

Who is this for?
-----------------
This is example code for advanced users of OpenSim who have a pretty decent understanding of the OpenSim and Simbody API and who feel confident with Matlab. 

It has a few useful code examples
- How to convert IMU data and use it in a tracking algorithm with OpenSim models.
- How to deal with Simbody quaternion and rotation tables.
- How to rotate data and generate new tables of data. 
- How to make (somewhat) custom Inverse Kinematics solutions using the InverseKinematicsSolver().
- How to utilize the Matlab Class structure for coding.

What do I need to run this?
-----------------
This code was used to prototype and test IMU tracking before we built C++ code 
in the latest version of OpenSim (4.1 at the time). There are functions and classes 
here that are not available in OpenSim 4.0 so you will have to either (i) wait 
for the 4.1 beta to be released or you can build the [OpenSim](https://github.com/opensim-org/opensim-core); from source.    

Why should I care?
-----------------
You don't need to really. The functions and methods found in this repo have been rolled into OpenSim already and you may find it easier to just use those tools as is. For me personally, going through the process of building an IMU tracking algorithm from the most basic OpenSim components was useful-- by sharing that code, others may be also find it useful.

Feel free to make comments or suggestions in the Github Issues tab above.

Getting Started
-----------------
As suggested, you need at least the current OpenSim (if building locally) or the downloadable OpenSim 4.1 distribution. The code was written using Matlab 2018b. To setup your OpenSim-Matlab environment, you can follow the instructions found [here](https://simtk-confluence.stanford.edu/display/OpenSim/Scripting+with+Matlab).

You can convert Xsens IMU data using the transform_imu_data_to_sto.m script. 

You can track the data with an OpenSim model using the run_IMU_inverse_kinematics.m.

Almost all the methods are in the orientationTracker.m class. 


See ya space cowboy
-James






