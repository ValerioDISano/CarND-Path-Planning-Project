#include "trajectory_planner.hpp"

void TrajectoryPlanner::setNextWaypoints(
        const double_vec& prev_traj_x,
        const double_vec& prev_traj_y)
{
    this->ref_pts_x.clear();
    this->ref_pts_y.clear();
  
    auto prev_size = prev_traj_x.size();
    
    if (prev_size < 2)
    {
        this->ref_pts_x.emplace_back(conf().currentX() - cos(conf().currentYaw()));
        this->ref_pts_x.emplace_back(conf().currentX());
        this->ref_pts_y.emplace_back(conf().currentY() - sin(conf().currentYaw()));
        this->ref_pts_y.emplace_back(conf().currentY());

    } else
    {
        std::copy(prev_traj_x.end()-2, prev_traj_x.end(),
                std::back_inserter(this->ref_pts_x));
        std::copy(prev_traj_y.end()-2, prev_traj_y.end(),
                std::back_inserter(this->ref_pts_y));

        auto updated_yaw = atan2(
                this->ref_pts_y.back()-this->ref_pts_y.rbegin()[1],
                this->ref_pts_x.back()-this->ref_pts_x.rbegin()[1]
                );
        conf().updateCarPose(this->ref_pts_x.back(),
                             this->ref_pts_y.back(),
                             updated_yaw);
    }

    size_t n_waypoints = 3;
    double s_increment = 30.0;
    for (auto index = 0; index < n_waypoints; index++)
    {
        auto wp = getXY(conf().currentS() + double(index + 1) * s_increment,
                conf().currentLane()*4 + 2,
                this->map_waypoints_s_,
                this->map_waypoints_x_,
                this->map_waypoints_y_
                );
        
        this->ref_pts_x.emplace_back(wp[0]);
        this->ref_pts_y.emplace_back(wp[1]);
    }
    
    // Coordinates transformation
    auto xi = this->ref_pts_x.begin();
    auto yi = this->ref_pts_y.begin();
    while (xi != this->ref_pts_x.end() and yi != this->ref_pts_y.end())
    {
        auto& x = *xi++;
        auto& y = *yi++;

        conf().fromGlobalToLocalCoordinates(x, y);
    }
}

void TrajectoryPlanner::computeNextTrajectory(
        const double_vec& prev_traj_x,
        const double_vec& prev_traj_y
        )
{    
   this->next_x.clear();
   this->next_x.reserve(this->n_pts_);
   this->next_y.clear();
   this->next_y.reserve(this->n_pts_);

   this->tp_spline.set_points(this->ref_pts_x, this->ref_pts_y);
   
   std::copy(prev_traj_x.begin(), prev_traj_x.end(), std::back_inserter(this->next_x)); 
   std::copy(prev_traj_y.begin(), prev_traj_y.end(), std::back_inserter(this->next_y)); 
   
   auto target_x = conf().targetX();
   auto target_y = this->tp_spline(target_x);
   auto target_mag = vectorMag2D(target_x, target_y);
   static double target_speed = 0.0;

   auto saturation = [](double& speed){
       if (speed > conf().maxSpeed())
        return conf().maxSpeed();
       else
        return speed;
   };

   double x_pre {0.0};
   for (auto i = 0; i < (this->n_pts_ - prev_traj_x.size()); i++)
   {
       target_speed += conf().plannedSpeedUp();
       target_speed = saturation(target_speed);
       double N_pts = target_mag / ((0.02 * target_speed) / 2.24);
       
       auto x_i = x_pre + target_x / N_pts;
       auto y_i = this->tp_spline(x_i);
       
       x_pre = x_i;

       conf().fromLocalToGlobalCoordinates(x_i, y_i);
       
       this->next_x.emplace_back(x_i);
       this->next_y.emplace_back(y_i);
   }
}
