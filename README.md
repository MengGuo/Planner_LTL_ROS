Agent-Planner-LTL-ROS
=================

ROS Interface for Temporal Task Planner

-----
Description
-----
This package contains an example interface between the temporal planner [P_MAS_TG](https://github.com/MengGuo/P_MAS_TG) and [ROS](http://www.ros.org). It serves as an on-line planner that receives motion and action confirmation from actuation modules and updates the next move for the robot.

<p align="center">  
  <img src="https://github.com/MengGuo/Planner_LTL_ROS/blob/master/src/figures/indep.png" width="600"/>
</p>

<p align="center">  
  <img src="https://github.com/MengGuo/Planner_LTL_ROS/blob/master/src/figures/multi.jpg" width="600"/>
</p>

-----
Features
-----
* Fully support any feature included in the [P_MAS_TG](https://github.com/MengGuo/P_MAS_TG) module.

* Easy integration with any motion capture system or low-level motion controller via ROS.

```python
# publish to actuation modular for motion and action
activity_pub = rospy.Publisher('next_move_%s' %letter, activity, queue_size=10)

# subscribe to motion and action module for confirmation 
rospy.Subscriber('activity_done_%s' %letter, confirmation, confirm_callback)

# subscribe to sensory module for workspace model update
rospy.Subscriber('knowledge_%s' %letter, knowledge, knowledge_callback)
```

* Two-cemera locationization system based on [ar_pose](http://wiki.ros.org/ar_pose), see [tf2pose_for_two.py](https://github.com/MengGuo/Planner_LTL_ROS/blob/master/src/tf2pose_for_two.py) for detials.


----
Usage
----
* [P_MAS_TG](https://github.com/MengGuo/P_MAS_TG) module. See details for how to build your own case there. 
* `catkin_make` in your `catkin_ws`
* Check [init.py](https://github.com/MengGuo/Planner_LTL_ROS/blob/master/src/init.py) for defining agent models.
* Run [planner.py](https://github.com/MengGuo/Planner_LTL_ROS/blob/master/src/planner.py) under ROS with the robot name as arugment. 


