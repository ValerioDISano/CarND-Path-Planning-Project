#ifndef BEHAVIOR_PLANNER_HPP_H
#define BEHAVIOR_PLANNER_HPP_H

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
          
          void updateCarFrenetCoord(const double& s, const double& d)
          {
              this->car_s = s;
              this->car_d = d;
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

          double car_s;
          double car_d;
          unsigned int lane;
    };
    

  private:

};

#endif
