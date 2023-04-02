
#include <a_star_planner/a_star_planner.h>

#include <pluginlib/class_list_macros.hpp>


//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(a_star_planner::AStarPlannerROS, nav_core::BaseGlobalPlanner)

namespace a_star_planner 
{

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

    AStarPlannerROS::Cell currentCell;

    std::priority_queue<AStarPlannerROS::Cell , std::vector<AStarPlannerROS::Cell>, 
                        std::function<decltype(AStarPlannerROS::heuristicCompare)>> frontier(AStarPlannerROS::heuristicCompare);

    std::vector<AStarPlannerROS::Cell> path;

    costmap_->worldToMap(start.pose.position.x, start.pose.position.y, startX, startY);
    costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goalX, goalY);

    currentCell.x = startX;
    currentCell.y = startY;
    currentCell.g = 0;
    currentCell.h = 0;
    currentCell.f = 0;

    while(currentCell.x != goalX && currentCell.y != goalY)
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

  bool AStarPlannerROS::heuristicCompare(AStarPlannerROS::Cell firstCell, AStarPlannerROS::Cell secondCell)
  {
    if(firstCell.f > secondCell.f)
    {
        return true;
    }
    else
    {
        return false;
    }
  }

  int computeHeuristic(AStarPlannerROS::Cell currentLocation, AStarPlannerROS::Cell goal)
  {
    int dx = std::abs(currentLocation.x - goal.x);
    int dy = std::abs(currentLocation.y - goal.y);
    int D = 1;
    int D2 = 1;
    return D * (dx + dy) + (D2 - 2 * D) * std::min(dx, dy);
  }
};