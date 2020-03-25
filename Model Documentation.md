# Path generation Reflection

The proposed path planning implementation is composed of two main actors:
    - Behavior planner
    - Trajectory planner

These two entites are modelled with two classes: BehaviorPlanner (bejavior_planner.hpp) and
TrajectoryPlanner (trajectory_planner.hpp).

## BehaviorPlanner

This is class is in charge to establish the behavior of the car taking into account the
data coming from the sensor fusion.
As first step a forecast of the scenario sourranding the car is made (predictionStep member function). In particular, during this step the algorithm tries to determine if there is a car in front or there are cars on the left or right side, whitin a range of 30m.

After the prediction phase, the behavior planner computes the actual behavior of the car for the
next iteration. By default the car will keep a constant velocity close to the speed limit
and will keep the central lane. If during the prediction step a car in front is found and there aren't cars on the left or on the right the car will overtake on the free lane. Otherwise, if there aren't free lanes, the car will slow down.

## TrajectoryPlanner

The trajectory is generated accordingly to the instructions of the behavior planner, namely the target lane and the accelertion. The trajectory computation is based on the computation of a spline.
The spline is computed on five points: the last two points from the previous trajectory (to avoid discontinuity problems) and three equidistant points setted in Frenet coordinates. Before calculating the spline, the points in Frenet coordinates are converted in Cartesian coordinates and all the points are transformed in vehicle local coordinates.

Once the spline is computed, the new trajectory is calculated by sampling the spline and 
transforming the points in global map coordinates. The points of the new trajectory are appended to the previous trajectory points that have not been consumed to ensure continuity.
