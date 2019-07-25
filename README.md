



# Simulation of kinematic/dynamic motion of rocking and walking objects.

This repository contains Matlab codes for simulation of the kinematic and dynamic rocking and walking object locomotion.
Rocking and walking object locomotion is a novel robotic manipulation firstly introduced in [manuscript](http://junseo.people.ust.hk/papers/rnw.pdf).
This manipulation technique is for transporting objects on the ground in a passive dynamic, nonprehensile manner. 
The object is manipulated to rock from side to side repeatedly; in the meantime, the force of gravity enables the object to roll along a zigzag path that is eventually heading forward.
For simultion of this motion, the object is modeled as an oblique circular cone. Considering the circular base of a conic object the mathematical equations governing the kinematic and 
dynamic of rolling falling disk is utilized for analysis of motion of the object. As such, the mathematical analysis of rolling and falling disk provided in [1] is exploited to write
the Matlab codes in this repository.


# Running the codes

The repository contains Matlab M-Files for each a brief description is provided as follow.

### MainCode.m:

The simulation may be started by opening the file "mainCode.m". In this file the properties of the conic object may be defined in the first part. 
Then four different scenarios for the simulation may be chosen as:
    
	1- Kinematic motion of the object when the apex point of the cone is fixed in space.
	
	2- Kinematic motion of the object when the apex point of the cone is not fixed in space.

    3- Dynamic motion of the object when the apex point of the cone is fixed in space. 

    4- Dynamic motion of the object when the apex point of the cone is not fixed in space.
	
For each scenario, the initial condition for the simulation is set. Then Matlab function ode45 is used to integrate a system of differential equations governing 
the kinematic and dynamic of the object. These systems of differential equations regarding to each scenario are defined by the function handle "SteadyStateFunction.m" 
which is called from the file "MainCode.m" by the ode45 Matlab function. 


### SteadyStateFunction.m:

The system of differential equations governing the kinematic and dynamic of conic object is defined in the file "SteadyStateFunction.m". The inputs of this file is 
the time span between initial and final time and the initial condition of the simulation. Other necessary parameters for the simulation are retrieved for the data files
 "robot.mat" and "sim_par.mat". The system of differential equations in this file are defined as the first order state space. For the kinematic simulation the dynamic of the
 object is not taken into account and the differential equations are derived according to constraints on the generalized coordinate. These constraints correspond
 to non-holonomic velocity constraints of rolling condition and the constraints correspond to whether the fixed or free apex point. The matrix related to these constraints is
 obtained from the "fun_Mat_a_const.m" which is a function of the generalized coordinate of the object.
 On the other hand, in dynamic simulation the matrices regarding to the dynamic of the object namely the matrices of inertia, vector of coriolis and vector of gravity are also
 needed which can be obtained from the file "dynamiceqpar.m". The inputs of this file are the general coordinate and velocity.
 
 
 
 
 
	
## References
[1] Jerry H. Ginsberg, "Advanced engineering dynamics", Cambridge University Press, 1998.




