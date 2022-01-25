# Thesis-Project
Repository related to the 3D Sim Kinematic and Dynamic

pk-EP v.1.0.0:
  - First Implementation regarding the Kinematic Sim
  - Implementation of Euler Parameters instead of Bryant Angles ( to avoid Singularities);
      - Implementation of the Velocity and Acceleration Equations
      - Implementation of SkewMatrix Modules, PerpendicularVectors and UnitVectors

pk-EP v.1.0.1:
  - Implementation of the condition number for the inverse kinematics (IK) linear systems (velocity and acceleration);
  - Restructure to the pre processing data module:
      - To be capable of also reading rotation inputs and not only vectors and points inputs;
  - Implementation of a 3D Example for Debug of lsqr and fsolver;
