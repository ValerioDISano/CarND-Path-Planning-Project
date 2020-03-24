#include "behavior_planner.hpp"

void BehaviorPlanner::computeBehavior() const
{
    int current_lane = conf().currentLane();
    // target lane
    if (this->last_prediction.car_ahead)
    {
    // evaluate a takeover
        if (current_lane > 0 && !this->last_prediction.car_on_left)
        {   // Left
            conf().changeLaneToLeft();        
        } else if (current_lane < 2 && !this->last_prediction.car_on_right)
        {   // Right
            conf().changeLaneToRight();        
        } else
        {
            conf().slowDown();
        }
    } else  // evaluate to speedup and to go back to the default lane
    {
       if (conf().currentSpeed() < conf().maxSpeed())
            conf().speedUp();
       if (( current_lane == 0 && !this->last_prediction.car_on_right ) ||
               ( current_lane == 2 && !this->last_prediction.car_on_left ))
       {
            conf().defaultLane();
       }
    }
}

void BehaviorPlanner::VehicleConfiguration::fromLocalToGlobalCoordinates(
        double& local_x, double& local_y
        )
{
        double local_x_tmp {local_x};
        double local_y_tmp {local_y};

        local_x = this->car_x_ +
            (local_x_tmp * cos(this->car_yaw_) - local_y_tmp * sin(this->car_yaw_));
        local_y = this->car_y_ +
            (local_x_tmp * sin(this->car_yaw_) + local_y_tmp * cos(this->car_yaw_));
}

void BehaviorPlanner::VehicleConfiguration::fromGlobalToLocalCoordinates(
        double& global_x, double& global_y
        )
{
        double translated_x = global_x - this->car_x_;
        double translated_y = global_y - this->car_y_;

        global_x = translated_x * cos(this->car_yaw_) + translated_y * sin(this->car_yaw_);
        global_y = translated_y * cos(this->car_yaw_) - translated_x * sin(this->car_yaw_);
}
