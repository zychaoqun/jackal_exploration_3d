# README #

This is a autonomous exploration implementation using the method in:

S. Bai, J. Wang, F. Chen, and B. Englot, "Information-Theoretic Exploration with Bayesian Optimization," IEEE/RSJ International Conference on Intelligent Robots and Systems(IROS), October 2016.


* Version  0.5

### How do I get set up? ###

* Compiling:

```
my_catkin_workspace/src$ git clone
my_catkin_workspace/src$ cd ..
my_catkin_workspace$ catkin_make
```

* Running:

From Jackal:
```
$ roslaunch jackal_planner gmapping_jackal.launch
$ rosrun jackal_exploration_3d jackal_exploration_da 
```
From Remote Laptop:
```
$ rosrun rviz rviz
```

### Contributor ###
Shi Bai(Bona), Tixiao Shan, Xiangyu Xu(Shawn)
