#ifndef A_STAR_LOCAL_PLANNER_A_STAR_PLANNER_ROS_H_
#define A_STAR_LOCAL_PLANNER_A_STAR_PLANNER_ROS_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

#include <nav_core/base_global_planner.h>

#include <vector>
#include <queue>

namespace a_star_planner {
  class AStarPlannerROS : public nav_core::BaseGlobalPlanner
  {
    public:
      AStarPlannerROS();

      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

      ~AStarPlannerROS();
  };
};
#endif