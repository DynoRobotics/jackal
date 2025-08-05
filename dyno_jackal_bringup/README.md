## Dyno jackal *(first draft)*

This README is meant to introduce you to how the Jackal works in the SIMLAN project.
The software in this repo is based on Clearpath Robotics own Jackal-repo, but is modified to fit the
specific needs of this project.


### Localization and Navigation

The Jackal uses the `Nav2`-plugin to autonomously navigate a pre-existing map.

All you have to do is make sure that `map_files` is correctly assigned in the
`nav2.launch.py` launch-file in the `dyno_jackal_bringup` package and that the jackal
is successfully loaded into the Rviz environment. 

* Remember that the map needs two files to work properly:

  - map_name.yaml *and*
  - map_name.pgm

  and that these need to exist in the same directory. (The .pgm file will be called upon by the yaml-file.)

`nav2` will automatically create and utilize a costmap from the provided map.

#### Monitoring

To properly monitor the navigation-process the following displays are recommended to use in Rviz:

* RobotModel -- Shows the robot model

* Map        -- For displaying map-related things
  - Map: For showing the picked map according to `map_files`
  - Global Costmap: Displays the costmap nav2 has calculated
  - Local Costmap: Not necessary but is an option

* Path       -- To show calculated paths
  - Plan: To show the path the robot will take towards its goal
  - Local Plan: To show the plan in the jackals closest proximity



#### If something crashed at start-up
First make sure that there are no zombie-processes running by running

```bash
ps aux
```

in the terminal. If there are zombie-processes (marked by 'Z' in the `STAT` column), rebuild the container.