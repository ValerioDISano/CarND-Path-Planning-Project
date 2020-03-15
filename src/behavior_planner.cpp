#include "behavior_planner.hpp"

template <class SensorFusion>
Prediction BehaviorPlanner::predictionStep(const SensorFusion& sensor_fusion_data)
{
    Prediction pred;

    for(const auto& data : sensor_fusion_data)
    {
        auto traffic_car_d = data[6];
        auto traffic_car_s = data[5];

        int t_car_lane_id = Car::fromFrenet2LaneId(other_car_d);

        if (t_car_lane_id == -1) continue; //no traffic car close to our vehicle
        
        auto other_car_speed = vectorMag2D(data[3], data[4]);

        // should estimate current s traffic car position
        //
        int relative_car_position = t_car_lane_id - conf().currentLane();
        switch (relative_car_position)
        {
            case 0: pred.car_ahead = (traffic_car_s > conf().currentS());
                    break;
            case 1: pred.car_on_right = true;
                    break;
            case -1: pred.car_on_left = true;
                    break;
            default:
                    break;
        }
    }

    return pred;
}   
