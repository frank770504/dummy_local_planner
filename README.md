## Dummy Local Planner

### The purpose for creating this package is to write a minimal needed planner

 - This package is simplified by dwa_local_planner
 - I removed dwa relted code
 - I kept dynamic_reconfiguration and path publisher.
 - The dummy code is in dummy_planner_ros.h and .cpp
 - I still left dwa code in dummy_planner.h and .cpp
 - The usage about costmap and trajectory generator can see dwa code as refernce.
