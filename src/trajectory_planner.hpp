#ifndef TRAJECTORY_PLANNER_HPP_
#define TRAJECTORY_PLANNER_HPP_

#include <vector>
#include <utility> // pair, make_pair
#include <algorithm> //copy

#include "spline.h"
#include "behavior_planner.hpp"

using double_vec = std::vector<double>
using vector_pair = std::pair<double_vec&, double_vec&>
constexpr auto conf = &BehaviorPlanner::VehicleConfiguration::instance;

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
    {
        this->next_x (this->n_pts_);
        this->next_y (this->n_pts_);
        trajectory = std::make_pair(this->next_x, this->next_y)
    }

    void setNextWaypoints();
    vector_pair& computeNextTrajectory(
            const double_vec& prev_traj_x,
            const double_vec& prev_traj_y);

  private:

    double_vec ref_pts_x;
    double_vec ref_pts_y;
    vector_pair trajectory;

    tk::spline spline;

    size_t n_pts_; 

    double_vec next_x;
    dobule_vec next_y;

    double_vec& map_waypoints_x_;
    double_vec& map_waypoints_y_;
    double_vec& map_waypoints_s_;
};

#endif
