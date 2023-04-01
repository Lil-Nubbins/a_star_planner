#include <a_star_planner/a_star_planner_ros.h>

#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.hpp>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>

#include <nav_core/parameter_magic.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(a_star_planner::AStarPlannerROS, nav_core::BaseLocalPlanner)

namespace a_star_planner {

  AStarPlannerROS::AStarPlannerROS(){

  }

  void AStarPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) 
  {
    if(!initialized_)
    {
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();

      initialized_ = true;
    }
    else
    {
      ROS_WARN("This planner has already been initialized... doing nothing");
    }
  }

  bool AStarPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    return 0;
  }
  
  bool AStarPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    return 0;
  }

  bool AStarPlannerROS::isGoalReached() {
    return 0;
  }

  AStarPlannerROS::~AStarPlannerROS(){}
};