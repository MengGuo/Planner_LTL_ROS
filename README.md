Agent-Planner-LTL-ROS
=================

ROS Interface for Temporal Task Planner

-----
Description
-----
this package contains an example interface between the temporal planner https://github.com/MengGuo/P_MAS_TG and ROS.
It serves as an on-line planner that receives motion and action confirmation from actuation modules and updates the next move for the robot.

-----
Features
-----
* Fully support any feature included in the P_MAS_TG module.
* Easy integration with any motion capture system or low-level motion controller.

<p align="center">  
  <img src="https://github.com/MengGuo/Planner_LTL_ROS/blob/master/src/figures/indep.png" width="800"/>
</p>

<p align="center">  
  <img src="https://github.com/MengGuo/Planner_LTL_ROS/blob/master/src/figures/multi.jpg" width="800"/>
</p>

----
Usage
----
* ROS
* install python packages like networkx, ply
* ltlba_32 and ltlba_64 are executable files complied under Unix. For other OS, please visit http://www.lsv.ens-cachan.fr/%7Egastin/ltl2ba/download.php


