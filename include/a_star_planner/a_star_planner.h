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

namespace a_star_planner {
  class AStarPlanner
  {
    public:
      AStarPlanner();

      AStarPlanner(costmap_2d::Costmap2D* costmap_ros, std::string global_frame);

      ~AStarPlanner();

      bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, 
                    std::vector<geometry_msgs::PoseStamped>& plan);

      struct Cell 
      { 
        int x;
        int y;
        int g;
        int h;
        int f;
      };

      static bool heuristicCompare(Cell first_cell, Cell second_cell);
      
      int computeHeuristic(Cell current_location, Cell goal);

      costmap_2d::Costmap2D* costmap_;
      std::string global_frame_;
  };
};
#endif