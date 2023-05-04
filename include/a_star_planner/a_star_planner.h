#ifndef _A_STAR_PLANNER_H_
#define _A_STAR_PLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <nav_core/base_global_planner.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

#include <vector>
#include <queue>
#include <map>
#include <functional>
#include <utility>

#include <a_star_planner/cell.h>

namespace a_star_planner {
  class AStarPlanner
  {
    public:
      AStarPlanner();

      AStarPlanner(costmap_2d::Costmap2D* costmap_ros, std::string global_frame);

      ~AStarPlanner();

      bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, 
                    std::vector<geometry_msgs::PoseStamped>& plan);
      
    private:
      costmap_2d::Costmap2D* costmap_;
      std::string global_frame_;
  };
};
#endif