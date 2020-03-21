#ifndef BEHAVIOR_PLANNER_HPP_H
#define BEHAVIOR_PLANNER_HPP_H

#include "helpers.h"

constexpr auto conf = &BehaviorPlanner::VehicleConfiguration::instance;

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
          inline const double& currentYaw() const {return this->current_yaw_;}
          
          void SpeedUp() {this->planned_speedup_ += this->max_acc_;}
          void SlowDown() {this->planned_speedup_ -= this->max_acc_;}
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
            this->car_yaw_ = car_yaw_;
          }

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
    Prediction predictionStep(const SensorFusion& sensor_fusion_data); 
    void computeBehavior() const;

  private:
    Prediction last_prediction;
};

#endif
