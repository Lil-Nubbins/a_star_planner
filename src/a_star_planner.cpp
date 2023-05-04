#include <a_star_planner/a_star_planner.h>

namespace a_star_planner 
{

  AStarPlanner::AStarPlanner(){}

  AStarPlanner::AStarPlanner(costmap_2d::Costmap2D* costmap_ros, std::string global_frame)
  {
      costmap_ = costmap_ros;
      global_frame_ = global_frame;
  }

  bool AStarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, 
                std::vector<geometry_msgs::PoseStamped>& plan)
  {
    unsigned int start_x;
    unsigned int start_y;
    unsigned int goal_x;
    unsigned int goal_y;
    double distance_cost;
    double heuristic_cost;
    double total_cost;

    bool added = false;

    costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y);
    costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y);
    ROS_INFO("got coords %d, %d",start_x,start_y);

    Cell current_cell(start_x,start_y,0.0);
    Cell goal_cell(goal_x,goal_y,0.0);
    Cell temp_cell;
    ROS_INFO("filled start and goal");
    cost_so_far_[current_cell] = 0.0;
    ROS_INFO("filled maps");
    frontier_.push(current_cell);
    ROS_INFO("filled frontier");

    while(!frontier_.empty())
    {
      
      current_cell = frontier_.top();
      ROS_INFO("Current cell: %d, %d, %f",current_cell.getX(), current_cell.getY(), current_cell.getCost());
      frontier_.pop();

      if(current_cell == goal_cell)
      {
        break;
      }
      
      //check every index around the current cell
      for(int i=current_cell.getX()-1; i<=current_cell.getX()+1; i++)
      {
        for(int j=current_cell.getY()-1; j<=current_cell.getY()+1; j++)
        {
          //check if i/j are valid positions in the gridmap
          if((i >= 0 && i < costmap_->getSizeInCellsX()) && (j >= 0 && j < costmap_->getSizeInCellsY()))
          {
            //don't add the current cell to the frontier
            if(!(i == current_cell.getX() && j == current_cell.getY()))
            {
              //Only consider unknown or non-obstacle neighbors
              if(costmap_->getCost(i,j) <= 150 || costmap_->getCost(i,j) == 255)
              {
                temp_cell.setX(i);
                temp_cell.setY(j);

                distance_cost = cost_so_far_[current_cell]+1; //get the distance from the start cost
                heuristic_cost = temp_cell.computeHeuristic(goal_cell); //get the distance to the goal cost from heuristic
                total_cost = distance_cost+heuristic_cost;
                temp_cell.setCost(total_cost); //set the Cell cost to the combined cost for the priority queue

                //If this neighbor isn't known to us or is cheaper than the version of itself known to us, add it to our structures and track it.
                if(!cost_so_far_.count(temp_cell) || distance_cost < cost_so_far_[temp_cell])
                {
                  frontier_.push(temp_cell);
                  came_from_[temp_cell] = current_cell;
                  cost_so_far_[temp_cell] = distance_cost;
                }
              }
            }
          }
        }
      }
    }
    //convert to ROS format
    /*while(came_from[current_cell]!=NULL)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = global_frame_;
      
      costmap_->mapToWorld(current_cell->x, current_cell->y, pose.pose.position.x, pose.pose.position.y);

      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;

      plan.push_back(pose);
      current_cell = came_from[current_cell];
    }*/

    return true;
  }

  AStarPlanner::~AStarPlanner()
  {

  }
};