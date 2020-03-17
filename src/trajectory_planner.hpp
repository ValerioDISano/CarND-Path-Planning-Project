#ifndef TRAJECTORY_PLANNER_HPP_
#define TRAJECTORY_PLANNER_HPP_

#include <vector>

using double_vec = std::vector<double>

class TrajectoryPlanner
{

  public:

    TrajectoryPlanner(
            const double_vec& map_waypoints_x,
            const double_vec& map_waypoints_y,
            const double_vec& map_waypoints_s,
            const size_t n_pts=50) :
                map_waypoints_x_ {map_waypoints_x},
                map_waypoints_y_ {map_waypoints_y},
                map_waypoints_s_ {map_waypoints_s},
                n_pts_ {n_pts}
    {;}

  private:
    double_vec ref_pts_x;
    double_vec ref_pts_y;

    size_t n_pts_; 

    double_vec next_x;
    dobule_vec next_y;

    double_vec& map_waypoints_x_;
    double_vec& map_waypoints_y_;
    double_vec& map_waypoints_s_;
};

#endif
