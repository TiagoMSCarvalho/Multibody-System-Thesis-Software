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
