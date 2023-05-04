#ifndef A_STAR_PLANNER_ROS_H_
#define A_STAR_PLANNER_ROS_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

#include <nav_core/base_global_planner.h>

#include <vector>
#include <queue>

#include <a_star_planner/a_star_planner.h>

namespace a_star_planner {
  class AStarPlannerROS : public nav_core::BaseGlobalPlanner
  {
    public:
      AStarPlannerROS();

      AStarPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      AStarPlannerROS(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

      ~AStarPlannerROS();

      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

      bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, 
                    std::vector<geometry_msgs::PoseStamped>& plan);

      bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

      struct Cell 
      { 
        int x;
        int y;
        int g;
        int h;
        int f;
      };

    protected:
      bool initialized_;
      costmap_2d::Costmap2D* costmap_;
      ros::Publisher plan_pub_;
      AStarPlanner* planner_;

    private:
      std::string global_frame_;
      ros::ServiceServer make_plan_srv_;
  };
};
#endif