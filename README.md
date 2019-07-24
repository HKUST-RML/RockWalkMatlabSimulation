



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

### mainCode.m:

To start the simulation open this file "mainCode.m". In this file the properties of the conic object may be defined in the first part. Then four different simulation may be 
chosen to be performed as:
    
	1- Kinematic motion of the object when the apex point of the cone is fixed in space.
	
	2- Kinematic motion of the object when the apex point of the cone is not fixed in space.

    3- Dynamic motion of the object when the apex point of the cone is fixed in space. 

    4- Dynamic motion of the object when the apex point of the cone is not fixed in space.
	
## References
[1] Jerry H. Ginsberg, "Advanced engineering dynamics", Cambridge University Press, 1998.




