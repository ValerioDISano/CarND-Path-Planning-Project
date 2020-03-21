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
        
        this->ref_pts_x.emplace_back(wp[0]);
        this->ref_pts_y.emplace_back(wp[1]);
    }
}

vector_pair& TrajectoryPlanner::computeNextTrajectory(
        const double_vec& prev_traj_x,
        const double_vec& prev_traj_y
        )
{    
   this->next_x.clear();
   this->next_x.reserve(this->n_pts_);
   this->next_y.clear();
   this->next_y.reserve(this->n_pts_);

   this->tp_spline.set_points(this->ref_pts_x, this->ref_pts_y);
   
   std::copy(prev_traj_x.begin(), prev_traj_x.end(), this->next_x.begin()); 
   std::copy(prev_traj_y.begin(), prev_traj_y.end(), this->next_y.begin()); 
   
   auto target_x = conf().targetX();
   auto target_y = this->tp_spline(target_x);
   auto target_mag = vectorMag2D(target_x, target_y);
   auto target_speed = conf().currentSpeed();
   double N_pts = target_mag / ((0.02 * target_speed) / 2.24);

   for (auto i = 0; i < (this->n_pts_ - prev_traj_x.size()); i++)
   {
       auto x_i = target_x / N_pts;
       auto y_i = this->tp_spline(x_i);
       this->next_x.emplace_back(x_i);
       this->next_y.emplace_back(y_i);
   }

   return this->trajectory;
}
