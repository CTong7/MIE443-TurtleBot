# MIE443 Turtlebot Contest 2 Code

This repository contains our source code for our exploration and mapping algorithm for the Contest 2 of MIE443. 

### How to Use
To use this repository, clone it into the src folder of your catkin workspace and then do:

```
catkin_make
source devel/setup.bash
```

Then to run it, do:
```
rosrun mie443_contest2 contest2
```


The final output text file "box_results.txt" will be written inside of the directory where catkin_ws is stored. For example, if /home/catkin_ws is where your workspace is stored, then the text file path will be /home/box_results.txt


