#include <a_star_planner/a_star_planner.h>

#include <pluginlib/class_list_macros.hpp>

namespace a_star_planner 
{

  AStarPlanner::AStarPlanner()
  : costmap_(NULL){}

  AStarPlanner::AStarPlanner(costmap_2d::Costmap2D* costmap_ros)
    : costmap_(NULL) 
  {
      costmap_ = costmap_ros;
  }

  bool AStarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, 
                std::vector<geometry_msgs::PoseStamped>& plan)
  {
    unsigned int startX;
    unsigned int startY;
    unsigned int goalX;
    unsigned int goalY;

    AStarPlanner::Cell currentCell;
    AStarPlanner::Cell goalCell;
    AStarPlanner::Cell tempCell;

    std::priority_queue<AStarPlanner::Cell , std::vector<AStarPlanner::Cell>, 
                        std::function<decltype(AStarPlanner::heuristicCompare)>> frontier(AStarPlanner::heuristicCompare);

    std::vector<AStarPlanner::Cell> path;

    costmap_->worldToMap(start.pose.position.x, start.pose.position.y, startX, startY);
    costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goalX, goalY);

    currentCell.x = startX;
    currentCell.y = startY;
    currentCell.g = 0;
    currentCell.h = 0;
    currentCell.f = 0;

    goalCell.x = goalX;
    goalCell.y = goalY;
    
    path.push_back(currentCell);

    while(!(currentCell.x == goalX && currentCell.y == goalY))
    {

      if(currentCell.x-1 > 0)
      {
        if(costmap_->getCost(currentCell.x-1,currentCell.y) <= 150 || costmap_->getCost(currentCell.x-1,currentCell.y) == 255)
        {
          tempCell.x = currentCell.x-1;
          tempCell.y = currentCell.y;
          tempCell.g = currentCell.g+1;
          tempCell.h = computeHeuristic(tempCell, goalCell);
          tempCell.f = tempCell.g + tempCell.h;
          frontier.push(tempCell);
        }
      }
      if(currentCell.x+1 < costmap_->getSizeInCellsX())
      {
        if(costmap_->getCost(currentCell.x+1,currentCell.y) <= 150 || costmap_->getCost(currentCell.x+1,currentCell.y) == 255)
        {
          tempCell.x = currentCell.x+1;
          tempCell.y = currentCell.y;
          tempCell.g = currentCell.g+1;
          tempCell.h = computeHeuristic(tempCell, goalCell);
          tempCell.f = tempCell.g + tempCell.h;
          frontier.push(tempCell);
        }
      }
      if(currentCell.y-1 > 0)
      {
        if(costmap_->getCost(currentCell.x,currentCell.y-1) <= 150 || costmap_->getCost(currentCell.x,currentCell.y-1) == 255)
        {
          tempCell.x = currentCell.x;
          tempCell.y = currentCell.y-1;
          tempCell.g = currentCell.g+1;
          tempCell.h = computeHeuristic(tempCell, goalCell);
          tempCell.f = tempCell.g + tempCell.h;
          frontier.push(tempCell);
        }
      }
      if(currentCell.y+1 < costmap_->getSizeInCellsY())
      {
        if(costmap_->getCost(currentCell.x,currentCell.y+1) <= 150 || costmap_->getCost(currentCell.x,currentCell.y+1) == 255)
        {
          tempCell.x = currentCell.x;
          tempCell.y = currentCell.y+1;
          tempCell.g = currentCell.g+1;
          tempCell.h = computeHeuristic(tempCell, goalCell);
          tempCell.f = tempCell.g + tempCell.h;
          frontier.push(tempCell);
        }     
      }
      if(currentCell.x-1 > 0 && currentCell.y-1 > 0)
      {
        if(costmap_->getCost(currentCell.x-1,currentCell.y-1) <= 150 || costmap_->getCost(currentCell.x-1,currentCell.y-1) == 255)
        {
          tempCell.x = currentCell.x-1;
          tempCell.y = currentCell.y-1;
          tempCell.g = currentCell.g+1;
          tempCell.h = computeHeuristic(tempCell, goalCell);
          tempCell.f = tempCell.g + tempCell.h;
          frontier.push(tempCell);
        }  
      }
      if(currentCell.x-1 > 0 && currentCell.y+1 < costmap_->getSizeInCellsY())
      {
        if(costmap_->getCost(currentCell.x-1,currentCell.y+1) <= 150 || costmap_->getCost(currentCell.x-1,currentCell.y+1) == 255)
        {
          tempCell.x = currentCell.x-1;
          tempCell.y = currentCell.y+1;
          tempCell.g = currentCell.g+1;
          tempCell.h = computeHeuristic(tempCell, goalCell);
          tempCell.f = tempCell.g + tempCell.h;
          frontier.push(tempCell);
        }  
      }
      if(currentCell.x+1 < costmap_->getSizeInCellsX() && currentCell.y-1 > 0)
      {
        if(costmap_->getCost(currentCell.x+1,currentCell.y-1) <= 150 || costmap_->getCost(currentCell.x+1,currentCell.y-1) == 255)
        {
          tempCell.x = currentCell.x+1;
          tempCell.y = currentCell.y-1;
          tempCell.g = currentCell.g+1;
          tempCell.h = computeHeuristic(tempCell, goalCell);
          tempCell.f = tempCell.g + tempCell.h;
          frontier.push(tempCell);
        }  
      }
      if(currentCell.x+1 < costmap_->getSizeInCellsX() && currentCell.y+1 < costmap_->getSizeInCellsY())
      {
        if(costmap_->getCost(currentCell.x+1,currentCell.y+1) <= 150 || costmap_->getCost(currentCell.x+1,currentCell.y+1) == 255)
        {
          tempCell.x = currentCell.x+1;
          tempCell.y = currentCell.y+1;
          tempCell.g = currentCell.g+1;
          tempCell.h = computeHeuristic(tempCell, goalCell);
          tempCell.f = tempCell.g + tempCell.h;
          frontier.push(tempCell);
        }  
      }

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
        currentCell = frontier.top();
      
        frontier.pop();

        for(int i=0; i<path.size(); i++)
        {
          if(path[i].x==currentCell.x && path[i].y==currentCell.y)
          {
            exists = true;
          }
        }

        if(!exists)
        {
          path.push_back(currentCell);
          added=true;
        }
      }
      
      frontier = std::priority_queue<AStarPlanner::Cell , std::vector<AStarPlanner::Cell>, 
                        std::function<decltype(AStarPlanner::heuristicCompare)>>(AStarPlanner::heuristicCompare);

    }

    return true;
  }

  AStarPlanner::~AStarPlanner()
  {

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
};