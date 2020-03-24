#ifndef BEHAVIOR_PLANNER_HPP_
#define BEHAVIOR_PLANNER_HPP_

#include <iostream>
#include "helpers.h"

class BehaviorPlanner
{
  public:
    
    class VehicleConfiguration
    {
      public:
          VehicleConfiguration(const VehicleConfiguration&) = delete;
          VehicleConfiguration(VehicleConfiguration&&) = delete;
          VehicleConfiguration& operator=(const VehicleConfiguration&) = delete;
          VehicleConfiguration& operator=(VehicleConfiguration&&) = delete;

          inline const double& maxSpeed() const {return this->max_speed_;}
          inline const double& maxAcc() const {return this->max_acc_;}
          inline const double& currentS() const {return this->car_s_;}
          inline const double& currentD() const {return this->car_d_;}
          inline const int& currentLane() const {return this->lane_;}
          inline const double& targetX() const {return this->target_x_;}
          inline const double& currentSpeed() const {return this->current_speed_;}
          inline const double& currentYaw() const {return this->car_yaw_;}
          
          void speedUp() {this->planned_speedup_ += this->max_acc_;}
          void slowDown() {this->planned_speedup_ -= this->max_acc_;}
          void changeLaneToLeft() {this->lane_--;}
          void changeLaneToRight() {this->lane_++;}
          void defaultLane() {this->lane_ = 1;}
          void updateCarFrenetCoord(const double& s, const double& d)
          {
              this->car_s_ = s;
              this->car_d_ = d;
          }

          void updateCarPose(
                    const double& x,
                    const double& y,
                    const double& yaw
                  )
          {
            this->car_x_ = x;
            this->car_y_ = y;
            this->car_yaw_ = yaw;
          }
          
          void fromLocalToGlobalCoordinates(double& local_x, double& local_y);
          void fromGlobalToLocalCoordinates(double& global_x, double& global_y);

          static VehicleConfiguration& instance()
          {
            static VehicleConfiguration conf {};
            return conf;
          }
      private:
          VehicleConfiguration() {;};
          
          const double max_speed_ {40.0};
          const double max_acc_ {0.2};

          double car_s_;
          double car_d_;
          double car_x_;
          double car_y_;
          double car_yaw_;
          double planned_speedup_ {0.0};
          double current_speed_ {0.0};
          const double target_x_ {0.0};
          int lane_;
    };

    typedef struct Prediction
    {
        bool car_ahead;
        bool car_on_left;
        bool car_on_right;

        Prediction() : car_ahead{false}, car_on_left{false}, car_on_right{false} {;}

    } Prediction;
    
    template <class SensorFusion>
    Prediction predictionStep(const SensorFusion& sensor_fusion_data, const std::size_t& prev_steps)
    {
    Prediction pred;

        for(const auto& data : sensor_fusion_data)
        {
            double traffic_car_d = data[6];
            double traffic_car_s = data[5];

            int t_car_lane_id = Car::fromFrenet2LaneId(traffic_car_d);

            if (t_car_lane_id == -1) continue; //no traffic car close to our vehicle
            
            auto traffic_car_speed = vectorMag2D(double(data[3]), double(data[4]));

            // estimate current s traffic car position
            traffic_car_s += (double(prev_steps) * 0.02 * traffic_car_speed); 
            
            int relative_car_position = t_car_lane_id - conf().currentLane();
            auto ego_car_s = conf().currentS();

            switch (relative_car_position)
            {
                case 0: pred.car_ahead |= (traffic_car_s > ego_car_s) &&
                        (traffic_car_s - ego_car_s < 30.0);
                        break;
                case 1: pred.car_on_right |= isContained(ego_car_s,
                                traffic_car_s - 30.0,
                                traffic_car_s + 30.0);
                        break;
                case -1: pred.car_on_left |= isContained(ego_car_s,
                                 traffic_car_s - 30.0,
                                 traffic_car_s + 30.0);
                        break;
                default:
                        std::cout << "Invalid realative car position!" << "\n";
                        break;
            }
        }
    
        this->last_prediction = pred;
        return pred;
    }
    
    void computeBehavior() const;

  private:
    Prediction last_prediction;
    static constexpr auto conf = &BehaviorPlanner::VehicleConfiguration::instance;
};

#endif
