
#include <a_star_planner/a_star_planner.h>

#include <pluginlib/class_list_macros.hpp>


//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(a_star_planner::AStarPlannerROS, nav_core::BaseGlobalPlanner)

namespace a_star_planner {

  AStarPlannerROS::AStarPlannerROS()
  : costmap_(NULL), initialized_(false){}

  AStarPlannerROS::AStarPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_(NULL),initialized_(false) 
  {
      //initialize the planner
      initialize(name, costmap_ros);
  }

  AStarPlannerROS::AStarPlannerROS(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame)
    : costmap_(NULL), initialized_(false) 
  {
      //initialize the planner
      initialize(name, costmap, global_frame);
  }

  void AStarPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
  }

  void AStarPlannerROS::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame)
  {
    if(!initialized_)
    {
      costmap_ = costmap;
      global_frame_ = global_frame;

      ros::NodeHandle private_nh("~/" + name);

      plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

      private_nh.param("default_tolerance", default_tolerance_, 2.0);

      make_plan_srv_ =  private_nh.advertiseService("make_plan", &AStarPlannerROS::makePlanService, this);

      initialized_ = true;
    }
    else
    {
      ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
    }
  }

  bool AStarPlannerROS::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, 
                std::vector<geometry_msgs::PoseStamped>& plan)
  {
    unsigned int startX;
    unsigned int startY;
    unsigned int goalX;
    unsigned int goalY;
    int g = 0;
    int h = 0;
    int f = 0;


    std::pair<std::pair<int,int>,int> currentCell;

    std::priority_queue<std::pair<std::pair<int,int>,int> , std::vector<std::pair<std::pair<int,int>,int>>, 
                        std::function<decltype(AStarPlannerROS::heuristicCompare)>> frontier(AStarPlannerROS::heuristicCompare);

    std::vector<std::pair<std::pair<int,int>,int>> path;

    costmap_->worldToMap(start.pose.position.x, start.pose.position.y, startX, startY);
    costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goalX, goalY);

    currentCell = {{startX,startY},g};

    while(!(std::get<0>(std::get<0>(currentCell))!= goalX && std::get<1>(std::get<1>(currentCell))!= goalY ))
    {
      path.push_back(currentCell);


    }


    return 0;
  }

  bool AStarPlannerROS::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp)
  {
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = global_frame_;

    return true;
  }

  AStarPlannerROS::~AStarPlannerROS()
  {

  }

  bool AStarPlannerROS::heuristicCompare(std::pair<std::pair<int,int>,int> firstCell, std::pair<std::pair<int,int>,int> secondCell)
  {
    if(std::get<1>(firstCell) > std::get<1>(secondCell))
    {
        return true;
    }
    else
    {
        return false;
    }
  }

  int computeHeuristic(std::pair<int,int> currentLocation, std::pair<int,int> goal)
  {
    int dx = std::abs(std::get<0>(currentLocation) - std::get<0>(goal));
    int dy = std::abs(std::get<1>(currentLocation) - std::get<1>(goal));
    int D = 1;
    int D2 = 1;
    return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)
  }
};