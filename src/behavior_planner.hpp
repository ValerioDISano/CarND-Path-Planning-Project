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

          static VehicleConfiguration& instance()
          {
            static VehicleConfiguration conf {};
            return conf;
          }
      private:
          VehicleConfiguration() {;};
          
          const double max_speed_ {40.0};
          const double max_acc_ {0.2};
    };
    

  private:

};

#endif
