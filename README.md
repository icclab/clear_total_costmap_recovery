# Clear Total Costmap Recovery

A ROS package that exports the ROS plugin clear_total_costmap_recovery. This recovery behavior plugin reverts all costmaps (global and local) to the static layer (essentially calls the ROS service /move_base/clear_costmaps). To add this plugin simply clone this repo into a catkin workspace, build workspace, and add the following line as part of `recovery_behaviors` in `move_base_params.yaml`.

````
  - name: 'super_aggressive_reset'
    type: 'clear_total_costmap_recovery/ClearTotalCostmapRecovery'
```` 
Check http://wiki.ros.org/pluginlib for more info on ROS plugins.
