
#include <a_star_planner/a_star_planner_ros.h>

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

      make_plan_srv_ =  private_nh.advertiseService("make_plan", &AStarPlannerROS::makePlanService, this);

      planner_ = new AStarPlanner(costmap, global_frame);

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
    planner_->makePlan(start, goal, plan);

    nav_msgs::Path output;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = global_frame_;
    output.poses = plan;

    plan_pub_.publish(output);
    return true;
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
};