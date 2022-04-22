# Thesis-Project
Repository related to the 3D Sim Kinematic and Dynamic

pk-EP v.1.0.0:
  - First Implementation regarding the Kinematic Sim
  - Implementation of Euler Parameters instead of Bryant Angles ( to avoid Singularities);
      - Implementation of the Velocity and Acceleration Equations
      - Implementation of SkewMatrix Modules, PerpendicularVectors and UnitVectors

pk-EP v.1.0.1:
  - Implementation of the condition number for the inverse kinematics (IK) linear systems (velocity and acceleration);
  - Restructure to the pre processing data module, three new types of inputs to define the body orientation:
      - Axis Vectors, the user defines 2 axis of the body frame;
      - Bryant Angles, the user supplies the bryant angles corresponding to the rotation of the frame (x-y-z order);
      - Orientational Axis, the user define a vector-axis around which the frame rotates (active view) and a angle corresponding to the rotation;
  - Implementation of a 3D Example for Debug of lsqr and fsolver;
  - Version Closure: 
	- Statement:
		- Closed for backup porpuses;
		- An extensive debug was done during the week, several bugs were found and corrected at this time the comparison with ADAMS is the following:
			- Translational joint:
				- All values are correct since only Vx is different from 0;
				- Velocity signals are all correct;
				- Velocity values of Matlab are higher than expected;
				- Acceleration values are different from expected;
			- Universal Joint:
				- Velocity values are erratic, Vy should be 0;
				- Acceleration Values don't add up vs Adams, Accel in y should be 0, Ang Accel in Z have different magnitudes;
			- Spherical Joint:
				- Velocity values are correct both in x and y;
				- Angular z velocity signal is wrong, values are different than expected;
				- Acceleration values are wrong;

pk-EP v.1.0.2:
  - Implementation of debug changes to the joints equations, experience changing some inconsistencies between the Wichita thesis and Nikravesh.
  - Issues Detected:
	- Acceleration Values were wrong in the joints and center of mass;
		- Solved: Issue with the Driver Input Equation ( Nikravesh page 175(193) equation 6.112)
			- Attention: The program is only giving the correct values with the signal of the equation second term is positive.
			- Simulations were done for positive and negative values of the angular velocity both are correct.
	- The y component of the velocity in the center of mass of body 3 is not giving the correct values
		- May be problematic due to the dynamic simulation.

pk-EP v1.0.3:
  - Implementation of a more robust input function:
  	- Implementation of the Spherical-Spherical Composite Joint;
   	- Implementation of a new Driver Equation Input:
		- Program now reads handle functions from the excel;
		- Program has lost "sturdiness" in relation to the Rotational Inputs being in Rads or Deg; [Limitation]
			- The sturdiness is maintained for the Body Orientation Input;
	-  Displacements and rotations are being applied to the CoM of the body, not to the point of application
	- Version CLosure:
		- SPH - SPH Implemented
		- Sinusoidal and Polynomial input functions implemented 

mbs-EP v1.0.0:
  - Implementation of the dynamic simulations:
  	- Solves the linear initial acceleration problem with the input of the q0 and rd0/w0 or q and rd/w for t0+timestep;
  	- Implements the ODE45 function (runge-Kutta algorithm with a method of 4th Order and Error Estimation of 5th Order) to integrate the DAE's of the velocity and acceleration;
  	- Uses the algorithm of Direct Correction by S.Yoon to geometric eliminate the constraint violations of the joints;
  - The program is capable of receiving:
  	-  Linear and Non Linear Spring and Damper Functions (Up to 3 functions for 1 spring);
  	-  Capable of receiving Sinusoidal and Polynomial Driving Input Functions for Position, Velocity and Acceleration;
  	-  Run simulations in mmks and MKS (SI) Units;
  	-  Run simulations with gravity or without it;
