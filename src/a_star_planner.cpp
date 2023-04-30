#include <a_star_planner/a_star_planner.h>

namespace a_star_planner 
{

  AStarPlanner::AStarPlanner()
  : costmap_(NULL){}

  AStarPlanner::AStarPlanner(costmap_2d::Costmap2D* costmap_ros, std::string global_frame)
    : costmap_(NULL) 
  {
      costmap_ = costmap_ros;
      global_frame_ = global_frame;
  }

  bool AStarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, 
                std::vector<geometry_msgs::PoseStamped>& plan)
  {
    unsigned int startX;
    unsigned int startY;
    unsigned int goalX;
    unsigned int goalY;

    AStarPlanner::Cell current_cell;
    AStarPlanner::Cell goal_cell;
    AStarPlanner::Cell temp_cell;

    std::priority_queue<AStarPlanner::Cell , std::vector<AStarPlanner::Cell>, 
                        std::function<decltype(AStarPlanner::heuristicCompare)>> frontier(AStarPlanner::heuristicCompare);

    std::vector<AStarPlanner::Cell> path;

    costmap_->worldToMap(start.pose.position.x, start.pose.position.y, startX, startY);
    costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goalX, goalY);

    current_cell.x = startX;
    current_cell.y = startY;
    current_cell.g = 0;
    current_cell.h = 0;
    current_cell.f = 0;

    goal_cell.x = goalX;
    goal_cell.y = goalY;
    
    path.push_back(current_cell);

    while(!(current_cell.x == goalX && current_cell.y == goalY))
    {
      fillFrontier(current_cell, goal_cell, &frontier);

      bool exists = false;
      bool added = false;

      if(frontier.size()==0)
      {
        return false;
      }

      while(added==false)
      {

        if(frontier.size()==0)
        {
          return false;
        }

        exists = false;
        current_cell = frontier.top();
      
        frontier.pop();

        for(int i=0; i<path.size(); i++)
        {
          if(path[i].x==current_cell.x && path[i].y==current_cell.y)
          {
            exists = true;
          }
        }

        if(!exists)
        {
          path.push_back(current_cell);
          added=true;
        }
      }
      
      frontier = std::priority_queue<AStarPlanner::Cell , std::vector<AStarPlanner::Cell>, 
                        std::function<decltype(AStarPlanner::heuristicCompare)>>(AStarPlanner::heuristicCompare);

    }

    for(int i = 0; i< path.size(); i++)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = global_frame_;
      
      costmap_->mapToWorld(path[i].x, path[i].y, pose.pose.position.x, pose.pose.position.y);

      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;

      plan.push_back(pose);
    }

    return true;
  }

  void AStarPlanner::fillFrontier(Cell current_cell, Cell goal_cell, std::priority_queue<AStarPlanner::Cell , std::vector<AStarPlanner::Cell>, 
                        std::function<decltype(AStarPlanner::heuristicCompare)>>* frontier)
  {
    AStarPlanner::Cell temp_cell;

    if(current_cell.x-1 > 0)
      {
        if(costmap_->getCost(current_cell.x-1,current_cell.y) <= 150 || costmap_->getCost(current_cell.x-1,current_cell.y) == 255)
        {
          temp_cell.x = current_cell.x-1;
          temp_cell.y = current_cell.y;
          temp_cell.g = current_cell.g+1;
          temp_cell.h = computeHeuristic(temp_cell, goal_cell);
          temp_cell.f = temp_cell.g + temp_cell.h;
          frontier->push(temp_cell);
        }
      }
      if(current_cell.x+1 < costmap_->getSizeInCellsX())
      {
        if(costmap_->getCost(current_cell.x+1,current_cell.y) <= 150 || costmap_->getCost(current_cell.x+1,current_cell.y) == 255)
        {
          temp_cell.x = current_cell.x+1;
          temp_cell.y = current_cell.y;
          temp_cell.g = current_cell.g+1;
          temp_cell.h = computeHeuristic(temp_cell, goal_cell);
          temp_cell.f = temp_cell.g + temp_cell.h;
          frontier->push(temp_cell);
        }
      }
      if(current_cell.y-1 > 0)
      {
        if(costmap_->getCost(current_cell.x,current_cell.y-1) <= 150 || costmap_->getCost(current_cell.x,current_cell.y-1) == 255)
        {
          temp_cell.x = current_cell.x;
          temp_cell.y = current_cell.y-1;
          temp_cell.g = current_cell.g+1;
          temp_cell.h = computeHeuristic(temp_cell, goal_cell);
          temp_cell.f = temp_cell.g + temp_cell.h;
          frontier->push(temp_cell);
        }
      }
      if(current_cell.y+1 < costmap_->getSizeInCellsY())
      {
        if(costmap_->getCost(current_cell.x,current_cell.y+1) <= 150 || costmap_->getCost(current_cell.x,current_cell.y+1) == 255)
        {
          temp_cell.x = current_cell.x;
          temp_cell.y = current_cell.y+1;
          temp_cell.g = current_cell.g+1;
          temp_cell.h = computeHeuristic(temp_cell, goal_cell);
          temp_cell.f = temp_cell.g + temp_cell.h;
          frontier->push(temp_cell);
        }     
      }
      if(current_cell.x-1 > 0 && current_cell.y-1 > 0)
      {
        if(costmap_->getCost(current_cell.x-1,current_cell.y-1) <= 150 || costmap_->getCost(current_cell.x-1,current_cell.y-1) == 255)
        {
          temp_cell.x = current_cell.x-1;
          temp_cell.y = current_cell.y-1;
          temp_cell.g = current_cell.g+1;
          temp_cell.h = computeHeuristic(temp_cell, goal_cell);
          temp_cell.f = temp_cell.g + temp_cell.h;
          frontier->push(temp_cell);
        }  
      }
      if(current_cell.x-1 > 0 && current_cell.y+1 < costmap_->getSizeInCellsY())
      {
        if(costmap_->getCost(current_cell.x-1,current_cell.y+1) <= 150 || costmap_->getCost(current_cell.x-1,current_cell.y+1) == 255)
        {
          temp_cell.x = current_cell.x-1;
          temp_cell.y = current_cell.y+1;
          temp_cell.g = current_cell.g+1;
          temp_cell.h = computeHeuristic(temp_cell, goal_cell);
          temp_cell.f = temp_cell.g + temp_cell.h;
          frontier->push(temp_cell);
        }  
      }
      if(current_cell.x+1 < costmap_->getSizeInCellsX() && current_cell.y-1 > 0)
      {
        if(costmap_->getCost(current_cell.x+1,current_cell.y-1) <= 150 || costmap_->getCost(current_cell.x+1,current_cell.y-1) == 255)
        {
          temp_cell.x = current_cell.x+1;
          temp_cell.y = current_cell.y-1;
          temp_cell.g = current_cell.g+1;
          temp_cell.h = computeHeuristic(temp_cell, goal_cell);
          temp_cell.f = temp_cell.g + temp_cell.h;
          frontier->push(temp_cell);
        }  
      }
      if(current_cell.x+1 < costmap_->getSizeInCellsX() && current_cell.y+1 < costmap_->getSizeInCellsY())
      {
        if(costmap_->getCost(current_cell.x+1,current_cell.y+1) <= 150 || costmap_->getCost(current_cell.x+1,current_cell.y+1) == 255)
        {
          temp_cell.x = current_cell.x+1;
          temp_cell.y = current_cell.y+1;
          temp_cell.g = current_cell.g+1;
          temp_cell.h = computeHeuristic(temp_cell, goal_cell);
          temp_cell.f = temp_cell.g + temp_cell.h;
          frontier->push(temp_cell);
        }  
      }
  }

  bool AStarPlanner::heuristicCompare(AStarPlanner::Cell first_cell, AStarPlanner::Cell second_cell)
  {
    if(first_cell.f > second_cell.f)
    {
        return true;
    }
    else
    {
        return false;
    }
  }

  int AStarPlanner::computeHeuristic(AStarPlanner::Cell current_location, AStarPlanner::Cell goal)
  {
    int dx = std::abs(current_location.x - goal.x);
    int dy = std::abs(current_location.y - goal.y);
    int D = 1;
    int D2 = 1;
    return D * (dx + dy) + (D2 - 2 * D) * std::min(dx, dy);
  }

  AStarPlanner::~AStarPlanner()
  {

  }
};