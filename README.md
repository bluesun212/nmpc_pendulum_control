## Demonstrating the swing-up and stabilization of a multi-link pendulum using Nonlinear Model Predicive Control (NMPC)
Over the past two decades, model predicive control has become one of the most useful and ubiquitous methods for the control of complex nonlinear systems.  This is fueled in part by the increasing computational power we have readily available and the advancement of efficient interior point methods for solving nonconvex optimization problems.  

The swing-up and stabilization of inverted pendulums has been a very common toy problem in the demonstration of control strategies for a long time due to the simplicity of implementation.  This project was my final project for Dr. James Rawling's MPC class and aims to demonstrate the performance of NMPC methods to swing up and stabilize a pendulum with an arbitrary number of linkages.  It relies on his group's package, [MPCTools](https://sites.engineering.ucsb.edu/~jbraw/software/mpctools/index.html) which itself uses CasADI for handling the nonlinear optimizations.   

### Description
1. The ```PendulumModel``` class uses the symbolic math toolbox to derive equations of motion of the multi-link cart-pendulum model.  It has a number of fields that represent various physical properties of the system.
2. The ```Simulation``` class interfaces with MPCTools and sets up and solves the MPC problem to get time series input and output data.
3. The ```Animation``` class uses the data from the ```Simulation``` class and creates an animation of the cart-pendulum model from the time series data.
4. The ```Logger``` class is an internal class used by the previous classes to convey solve progress to the user.

Finally, the ```project.m``` script demonstrates the swing-up and stabilization to 8 different equilibrium points of a triple pendulum.  Start with this file to see how to use the classes in this project.  This ```noise_sim.m``` script was used to explore how varying the number of linkages and the noise intensity affects the ability to stabilize the system.  

### Installation notes
First, install MPCTools and CasADI for use with MATLAB from the link above, then download the code from this project.  This project also needs the symbolic math toolbox.  
