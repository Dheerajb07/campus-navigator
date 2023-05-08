# Campus Navigator
## Overview
In this project various sampling-based motion planning algorithms were implemented to navigate WPI's campus using a 2D aerial map. 

First, a PRM algortihm with four different sampling methods - uniform, random, gaussian and bridge sampling methods, was implemented to construct roadmaps. The resulting roadmaps and paths were visualized and the effect of different sampling methods on roadmap generation was analyzed. 

In addition, RRT, RRT* and Informed-RRT* algorithms were implemented on the same map. Each of the three paths obtained for a query were evaluated using path length as a metric. It was seen that Informed-RRT* prodcues more optimal paths than RRT or RRT*.

### Code
* 'PRM Implementation' - code for PRM with different sampling methods
* 'RRT Implementation' - code for RRT, RRT* and Informed-RRT* 

Run 'main.py' in each folder. This script visualises the path found along with the roadmap/tree. The 'RRT.py' script has the code for the RRT, RRT* and Informed-RRT* algorithms and the 'PRM.py' contains the code for the PRM algorithm. The start and goal positions can be edited in 'main.py' along with the sampling parameters.

The 'results' folder contains the screenshots of the paths obtained.

This project is a combination of homeworks completed for RBE550-Motion Planning course at WPI. The 'doc' folder contains the writeups for these homeowrks.

## Probabilistic Road Map (PRM)
Uniform Sampling
<img src="PRM Implementation/results/uniform.png"/>

Random Sampling
<img src="PRM Implementation/results/random.png"/>

Gaussina Sampling
<img src="PRM Implementation/results/gaussian.png"/>

Bridge Sampling
<img src="PRM Implementation/results/bridge.png"/>

## Rapidly-Exploring Random Trees (RRT)
RRT
<img src="RRT Implementation/results/RRT.png"/>

RRT*
<img src="RRT Implementation/results/RRT_star.png"/>

Informed-RRT*
<img src="RRT Implementation/results/informed_rrt.png"/>

