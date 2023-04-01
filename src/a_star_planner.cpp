
#include <a_star_planner/a_star_planner.h>

#include <pluginlib/class_list_macros.hpp>


//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(a_star_planner::AStarPlannerROS, nav_core::BaseGlobalPlanner)

namespace a_star_planner {

  AStarPlannerROS::AStarPlannerROS()
  {

  }

  void AStarPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {

  }

  bool AStarPlannerROS::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, 
                std::vector<geometry_msgs::PoseStamped>& plan)
  {
    return 0;
  }

  bool AStarPlannerROS::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp)
  {
    return 0;
  }

  AStarPlannerROS::~AStarPlannerROS()
  {

  }
};