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
        }
    } else if (conf().currentSpeed() < conf().maxSpeed())  // evaluate to speedup
    {
       conf().speedUp(); 
    } else
    {
        conf().slowDown();
    }
}
