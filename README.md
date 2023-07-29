# Campus Navigator
## Overview
In this project, various sampling-based motion planning algorithms were implemented to navigate WPI's campus using a 2D aerial map. 

First, a PRM algorithm with four different sampling methods - uniform, random, gaussian, and bridge sampling methods, was implemented to construct roadmaps. The resulting roadmaps and paths were visualized and the effect of different sampling methods on roadmap generation was analyzed. 

In addition, RRT, RRT*, and Informed-RRT* algorithms were implemented on the same map. Each of the three paths obtained for a query was evaluated using path length as a metric. It was seen that Informed-RRT* produces more optimal paths than RRT or RRT*.

### Code
* `PRM Implementation` - code for PRM with different sampling methods
* `RRT Implementation` - code for RRT, RRT* and Informed-RRT* 

Run `main.py` in each folder. This script visualizes the path found along with the roadmap/tree. The `RRT.py` script has the code for the RRT, RRT*, and Informed-RRT* algorithms and the `PRM.py` contains the code for the PRM algorithm. The start and goal positions can be edited in `main.py` along with the sampling parameters.

The `results` folder contains screenshots of the paths obtained.

This project is a combination of homework completed for the RBE550-Motion Planning course at WPI. The `doc` folder contains the writeups for this homework.

## Probabilistic Road Map (PRM)
<em>Uniform Sampling: Path length - 263.53</em>

<img src="PRM Implementation/results/uniform.png" width = 450/>

<em>Random Sampling: Path length - 279.38</em>

<img src="PRM Implementation/results/random.png"  width = 450/>

<em>Gaussian Sampling: Path length - 285.36</em>

<img src="PRM Implementation/results/gaussian.png"  width = 450/>

<em>Bridge Sampling: Path length - 276.84</em>

<img src="PRM Implementation/results/bridge.png"  width = 450/>

## Rapidly-Exploring Random Trees (RRT)
<em>RRT: Path length - 356.16</em>

<img src="RRT Implementation/results/RRT.png"  width = 450/>

<em>RRT*: Path length - 254.92</em>

<img src="RRT Implementation/results/rrt_star.png"  width = 450/>

<em>Informed-RRT*: Path length - 250.20</em>

<img src="RRT Implementation/results/informed_rrt.png"  width = 450/>

