## New Function: dynamicDetector::publishDynamicObstacleVelocities()

This function publishes the calculated velocities of detected dynamic obstacles for external processing.
It publishes the topic `/dynamic_obstacle_velocity` (geometry_msgs/Twist) and displays an arrow above each dynamic obstacle to show its velocity.

```cpp
void dynamicDetector::publishDynamicObstacleVelocities() {
    // ...implementation code snippet...
}
```