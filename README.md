# coding_exercise
This repository is for presenting the solution to the coding exercise as part of the application process for a Senior Robotics Engineer role at BotsAndUs.

To meet with the pre-established requirements (functional and non-functional) of this exercise:
  - A ROS-Noetic compatible local planner has been implemented that provides
    a new local planner for move_base. The local planner (botsandus_planner) 
    implements the following features:
      1- Can receive path from the global planner and generates velocities 
        (linear and angular) to send to the base, as defined by the BaseLocalPlanner 
        API.
      2- The local planner always rotates in-place towards the next point in the path,
        and then move towards it.
      3- When arriving at the last point in the path, the planner should take teh supplied
        goal orientation by rotating in-place.
      4- The planner uses the costmap to detect obstacles and stop the robot 
        before collision occurs.
      5- A ROS Node (path_tracker_node) has been implemented in the path_tracker package
        that checks periodically if the robot is following the calculated path and
        publishes messages on two ROS topics: deviation error and if the robot appears 
        stuck and unable to follow the path.

## ASSUMPTIONS MADE
- It is assumed that the robot receives a static global path.
- It is assumed that we only want to track the inter-segment path deviation
  error of the mobile base.

## EXTRA FEATURES IMPLEMENTED
1- Path resolution downsampling: The local planner in order to function efficiently,
    allows the user to downsample (resolution reduction) a given planned path hence 
    reducing the number of stop-and-turns along the route as expected when navigating
    from point-to-point.
2- Ability to use both static and dynamic plans: Even though that assumption is made for
    static global plans, the local planner can also be customized to follow plans in an
    environment where the global planner keeps updating and changing the planned path.
3- Publishes waypoints in RViz for better visualization: An RViz visualization has been 
    added to allow for viewing the waypoints, the current robot deviation from the path
    and the robot base's status (moving or stuck).
4- Respects the robot's acceleration limits: The local planner in an attempt to follow 
    the planned path, also takes into account the robot's acceleration limits and generates
    only feasible velocities to move the robot base.
5- Two other packages: botsandus_localization and botsandus_navigation have been including
    for better organization of the stack.
    
## EXTRA PACKAGE DEPENDENCIES
- ros-noetic-jsk-rviz-plugins

## RUNNING THE PLANNER and TRACKING NODE
- To run the simulation stack use:
  roslaunch botsandus_navigation turtlebot3_navigation.launch
  
  This will bringup the following:
    - Gazebo Simulation with Turtlebot3 Burger
    - Robot State Publisher node
    - Map Server node
    - Move_base node configured to use the 'botsandus_planner' as its local planner
    - AMCL node for localization
    - RViz for visualization

- To run the path tracking node use:
  roslaunch path_tracker track_nav.launch

## UNIT TESTING
- To run the tests only for the "path_tracker" use:
  catkin_make run_tests_path_tracker
  
- To run the unit test for the 'local_planner' use:
  catkin_make run_tests_botsandus_planner 
  
  Time has not permitted me to fully test all the methods implemented in the 
  local planner. I have planned to write Mock Classes using gMock to help do away 
  with most of the dependencies required when writing unit tests. A most obvious 
  example would be to instantiate and populate costmaps which in this specific case,
  was the main reason for not writing unit tests for all the methods in the local 
  planner.
  
  Also, even though the code is heavily commented, I would have liked to use 
  Doxygen to extract all code comments and documentation into a single place, 
  presenting a uniform interface.
