# tars_control
The tars_control package provides control algorithms for the 3-wheeled, omni-directional "tars" rover. 

For all scripts, properly setting the radius of the wheels and the distance from the center of mass of the robot to the center of mass of the wheels is essential. These are float data types defined as rWheel and D respectively and are input in meters. These initializations occur at the beginning of each script.

<br/><br/>

## EXECUTABLES:

### Main:

*polynomialControl.cpp*

action server control algorithm which accepts polynomials created through mission planner to define its path. This node must be run through the following command: 
  
        rosrun tars_control polynomialControl
        
prior to launching the misson planner node. An output expressing that the server is waiting for the mission planner denotes that this rover controller is primed. The reference trajectory for PID control is determined through use of the position polynomials provided by the mission planner. The speeds are calculated by differentiating the position polynomials with respect to time.

### Extras:
*stop.cpp*

Executable to stop rover motion. Implemented to allow for stopping the robot during mission if desired. This node is run through the following command:

        rosrun tars_control stop

*circleControl.cpp*

Preset control algorithm which follows a circular trajectory defined with a radius and period. These can be altered within the scripts by editing the values for radius and Period in the initial section defining "Aspects of desired circular path". 
Uses vicon; Easy to switch to ORBSLAM due to equivalent data type. This node is run through the following command:
        
        rosrun tars_control circleControl


*semiCircleControl.cpp*

Preset control algorithm which follows a semi-circular trajectory defined with a radius and period. These can be altered within the scripts by editing the values for radius and Period in the initial section defining "Aspects of desired circular path". 
Uses vicon; Easy to switch to ORBSLAM due to equivalent data type. This node is run through the following command:
        
        rosrun tars_control semiCircleControl

<br/><br/>

## RESOURCES:
http://wiki.ros.org/

van Haendel, Rob PA. "Design of an omnidirectional universal mobile platform." RPA van Haedel.—Eindhoven.: Eindhoven University of Technology (2005).
