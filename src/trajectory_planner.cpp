#include "trajectory_planner.hpp"

void TrajectoryPlanner::setNextwWaypoints()
{
    size_t n_waypoints = 3;
    double s_increment = 30.0;
    for (auto& index = 0; index < n_waypoints; index++)
    {
        auto wp = getXY(conf().currentS() + i*s_increment,
                conf().currentLane()*4 + 2,
                this->map_waypoint_s_,
                this->map_waypoint_x_,
                this->map_waypoint_y_
                );
        
        this->next_x.emplace_back(wp[0]);
        this->next_y.emplace_back(wp[1]);
    }
}
