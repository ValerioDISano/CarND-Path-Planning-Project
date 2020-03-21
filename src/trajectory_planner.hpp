#ifndef TRAJECTORY_PLANNER_HPP_
#define TRAJECTORY_PLANNER_HPP_

#include <vector>
#include <utility> // pair, make_pair
#include <algorithm> //copy

#include "spline.h"
#include "behavior_planner.hpp"

using double_vec = std::vector<double>;
constexpr auto conf = &BehaviorPlanner::VehicleConfiguration::instance;

class TrajectoryPlanner
{

  public:

    TrajectoryPlanner(
            double_vec& map_waypoints_x,
            double_vec& map_waypoints_y,
            double_vec& map_waypoints_s,
            std::size_t n_pts=50) :
                map_waypoints_x_ {map_waypoints_x},
                map_waypoints_y_ {map_waypoints_y},
                map_waypoints_s_ {map_waypoints_s},
                n_pts_ {n_pts}
    {
        this->next_x = double_vec( n_pts );
        this->next_y = double_vec( n_pts );
    }

    inline double_vec& getTrajectoryX() {return this->next_x;}
    inline double_vec& getTrajectoryY() {return this->next_y;}
    
    void setNextWaypoints();
    void computeNextTrajectory(
            const double_vec& prev_traj_x,
            const double_vec& prev_traj_y);

  private:

    double_vec ref_pts_x;
    double_vec ref_pts_y;

    tk::spline tp_spline;

    std::size_t n_pts_; 

    double_vec next_x;
    double_vec next_y;

    double_vec& map_waypoints_x_;
    double_vec& map_waypoints_y_;
    double_vec& map_waypoints_s_;
};

#endif
