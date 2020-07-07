#include <jps_basis/data_utils.h>
#include <jps_planner/jps_planner/jps_planner.h>
#include <jps_planner/distance_map_planner/distance_map_planner.h>

#include <math.h>
#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

using namespace JPS;

class Plan2DToPath
{
public:
  Plan2DToPath();
private:
  void setUpJPS();
  void setUpDMP();
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  void waypointsCallback(const nav_msgs::Path::ConstPtr& msg);
  void publishMap();
  bool do_planning(geometry_msgs::PoseStamped& ps_start, geometry_msgs::PoseStamped& ps_goal, std::vector<nav_msgs::Path>& paths);
  void create_path2d(vec_Vec2f planner_path, nav_msgs::Path& path, const geometry_msgs::TransformStamped& transform, const ros::Time current_time, double path_z);
  bool computePlan(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res);

  ros::NodeHandle nh_, pnh_;

  ros::Publisher path_pub_, dmp_path_pub_, map_marker_pub_;
  ros::Subscriber map_sub_, odom_sub_, waypoints_sub_;
  nav_msgs::Odometry odom_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::ServiceServer plan_service_;

  std::shared_ptr<OccMapUtil> map_util_;
  bool map_initialized_;

  std::string world_frame_, planning_frame_, base_frame_;

  std::unique_ptr<JPSPlanner2D> planner_ptr_;
  std::unique_ptr<DMPlanner2D> dmplanner_ptr_;

  //mutex for changes of variables
  boost::mutex map_mutex_, odom_mutex_;
};

Plan2DToPath::Plan2DToPath(): tf_listener_(tf_buffer_)
{
  pnh_ = ros::NodeHandle("~");

  //~ We do planning in the frame of the yaml file or VoxelMap (not octomap)
  //~ For the yaml case, there are 3 frames, world (gazebo), map (octomap), and yaml (written map file, has to be shifted if the octomap has any negative values)
  //~ There is also a base_link for the robot local frame, but it is not needed by the planning (we use the ground_truth/odom topic in the world frame)
  //~ For the VoxelMap taken in by subscription case, there is the world (gazebo) and the map (local frame to the robot created by e.g. SLAM)
  pnh_.param<std::string>("world_frame", world_frame_, "world");
  pnh_.param<std::string>("base_frame", base_frame_, "base_link");
  pnh_.param<std::string>("planning_frame", planning_frame_, "voxel_map");

  map_initialized_ = false;

  map_util_ = std::make_shared<OccMapUtil>();

  //#TODO Setup planners, map_utils_ needs to be initialized, maybe init with empty?
  //setUpJPS();
  //setUpDMP();

  path_pub_ = nh_.advertise<nav_msgs::Path>("jps_path_2d", 1);
  dmp_path_pub_ = nh_.advertise<nav_msgs::Path>("dmp_path_2d", 1);
  map_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("map_check_2d", 1);

  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&Plan2DToPath::odomCallback, this, _1));
  waypoints_sub_ = nh_.subscribe<nav_msgs::Path>("waypoints", 2, boost::bind(&Plan2DToPath::waypointsCallback,  this, _1));
  map_sub_ = nh_.subscribe("projected_map", 1, &Plan2DToPath::mapCallback, this);

  plan_service_ = nh_.advertiseService("jps2d_plan_service", &Plan2DToPath::computePlan, this);
}

void Plan2DToPath::setUpJPS()
{
  planner_ptr_.reset(new JPSPlanner2D(true)); // Declare a planner
  planner_ptr_->setMapUtil(map_util_); // Set collision checking function
  planner_ptr_->updateMap();
}

void Plan2DToPath::setUpDMP()
{
  dmplanner_ptr_.reset(new DMPlanner2D(true)); // Declare a planner
  dmplanner_ptr_->setMap(map_util_,Vec2f(0.0, 0.0)); // Set collision checking function
  //~ dmplanner_ptr->updateMap();
  dmplanner_ptr_->setPotentialRadius(Vec2f(2.5, 2.5)); // Set 3D potential field radius
  dmplanner_ptr_->setSearchRadius(Vec2f(1.5, 1.5)); // Set the valid search region around given path
}

void Plan2DToPath::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(odom_mutex_);
  odom_ = *msg;
}

void Plan2DToPath::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  boost::mutex::scoped_lock lock(map_mutex_);

  Vec2f ori(msg->info.origin.position.x, msg->info.origin.position.y);
  Vec2i dim(msg->info.width, msg->info.height);
  decimal_t res = msg->info.resolution;
  std::vector<signed char> map = msg->data;
  planning_frame_ = msg->header.frame_id;

  map_util_->setMap(ori, dim, map, res);

  if (!map_initialized_)
  {
    map_initialized_ = true;
    ROS_INFO("Map initialized! frame %s", planning_frame_.c_str());
  }
  publishMap();
}

void Plan2DToPath::publishMap()
{
  const Vec2i dim = map_util_->getDim();
  //~ const Vec3f origin = map_util_->getOrigin();
  const double res = map_util_->getRes();
  visualization_msgs::Marker marker;
  marker.header.frame_id = planning_frame_;
  marker.header.stamp = ros::Time();
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.scale.x = res;
  marker.scale.y = res;
  marker.scale.z = res;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.points.reserve(dim(0)*dim(1));

  for(int x = 0; x < dim(0); x ++) {
    for(int y = 0; y < dim(1); y ++) {
      if(!map_util_->isFree(Vec2i(x, y))) {
        Vec2f pt = map_util_->intToFloat(Vec2i(x, y));
        geometry_msgs::Point point;
        point.x = pt(0);
        point.y = pt(1);
        point.z = 0.0;
        marker.points.emplace_back(point);
      }
    }
  }

  map_marker_pub_.publish(marker);
}

bool Plan2DToPath::computePlan(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res)
{
  ROS_INFO("JPS plan requested");

  geometry_msgs::PoseStamped ps_start = req.start;
  geometry_msgs::PoseStamped ps_goal = req.goal;

  ROS_INFO("%f %f %f %s",ps_start.pose.position.x,ps_start.pose.position.y,ps_start.pose.position.z, ps_start.header.frame_id.c_str());

  boost::mutex::scoped_lock lock(map_mutex_);
  {

  if (!map_initialized_)
  {
    ROS_ERROR("No map initialized");
    return false;
  }

  //#TODO does the planner needs to be reset everytime?
  setUpJPS();
  setUpDMP();

  std::vector<nav_msgs::Path> paths;
  bool ret = do_planning(ps_start, ps_goal, paths);

  if(ret)
  {
    ROS_INFO("Found valid plan");
    res.plan = paths[1]; //Set plan to be DMP
    return true;
  }
  else
    return false;
  }

  return false;
}

void Plan2DToPath::waypointsCallback(const nav_msgs::Path::ConstPtr& msg)
{
  geometry_msgs::PoseStamped ps_start, ps_goal;
  int wp_size = msg->poses.size();

  ROS_INFO("%d waypoints received", wp_size);
  if(wp_size > 2)
    ROS_WARN("First 2 waypoints treated as start, goal. have not yet implemented handling additional points");

  if(wp_size < 1)
  {
    ROS_ERROR("Must publish at least one waypoint");
    return;
  }
  if(wp_size == 1)
  {
    boost::mutex::scoped_lock lock(odom_mutex_);
    ps_start.pose = odom_.pose.pose;
    ps_start.header.frame_id = odom_.header.frame_id;
    ps_goal = msg->poses[0];
    ps_goal.header.frame_id = msg->header.frame_id;
    ROS_INFO("%f %f %f %s",ps_start.pose.position.x,ps_start.pose.position.y,ps_start.pose.position.z, ps_start.header.frame_id.c_str());
  }
  else
  {
    ps_start = msg->poses[0];
    ps_start.header.frame_id = msg->header.frame_id;
    ps_goal = msg->poses[1];
    ps_goal.header.frame_id = msg->header.frame_id;
  }

  std::vector<nav_msgs::Path> paths;

  boost::mutex::scoped_lock lock(map_mutex_);
  {
    if (!map_initialized_)
    {
      ROS_ERROR("No map initialized");
      return;
    }

    //#TODO does the planner needs to be reset everytime?
    setUpJPS();
    setUpDMP();

    bool ret = do_planning(ps_start, ps_goal, paths);
  }
}

void Plan2DToPath::create_path2d(vec_Vec2f planner_path, nav_msgs::Path& path, const geometry_msgs::TransformStamped& w_T_p, const ros::Time current_time, double path_z)
{
  path.header.frame_id = world_frame_;
  path.header.stamp = current_time;

  for(const auto& it: planner_path)
  {
    geometry_msgs::PoseStamped path_ps_planner, path_ps_world;
    path_ps_planner.header.stamp = current_time;
    path_ps_planner.header.frame_id = planning_frame_;
    path_ps_planner.pose.orientation.w = 1;
    path_ps_planner.pose.orientation.x = 0;
    path_ps_planner.pose.orientation.y = 0;
    path_ps_planner.pose.orientation.z = 0;

    path_ps_planner.pose.position.x = it.transpose()[0];
    path_ps_planner.pose.position.y = it.transpose()[1];
    path_ps_planner.pose.position.z = path_z;

    tf2::doTransform(path_ps_planner, path_ps_world, w_T_p); //Transform to world frame

    // linear interpolation to add points to path for ewok to run better
    if (path.poses.size() > 0)
    {
      float diff_x = path_ps_world.pose.position.x - path.poses.back().pose.position.x;
      float diff_y = path_ps_world.pose.position.y - path.poses.back().pose.position.y;

      int num_steps = int(sqrt(diff_x*diff_x+diff_y*diff_y)/0.2);
      geometry_msgs::PoseStamped interm_pt = path_ps_world;
      geometry_msgs::PoseStamped last_pt = path.poses.back();
      for (int i = 1; i < num_steps; i++)
      {
        interm_pt.pose.position.x = diff_x*i/num_steps + last_pt.pose.position.x;
        interm_pt.pose.position.y = diff_y*i/num_steps + last_pt.pose.position.y;
        interm_pt.pose.position.z = 0.0;
        path.poses.push_back(interm_pt);
      }
    }
    path.poses.push_back(path_ps_world);
  }
}

bool Plan2DToPath::do_planning(geometry_msgs::PoseStamped& ps_start, geometry_msgs::PoseStamped& ps_goal, std::vector<nav_msgs::Path>& paths)
{
  /*
  if (ps_goal.pose.position.x == ps_start.pose.position.x && ps_goal.pose.position.y == ps_start.pose.position.y && ps_goal.pose.position.z == ps_start.pose.position.z)
  {
    ROS_ERROR("Start and goal poses are the same. Have you published waypoints?");
    return;
  }*/
  ROS_INFO("Running planner");

  geometry_msgs::TransformStamped w_T_p, p_T_g;
  //ros::Time current_time = ros::Time::now();
  ros::Time current_time = ros::Time(0);
  try
  {
    w_T_p = tf_buffer_.lookupTransform(world_frame_, planning_frame_, current_time);
    p_T_g = tf_buffer_.lookupTransform(planning_frame_, ps_start.header.frame_id, current_time);
  }
  catch (tf2::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    return false;
  }

  geometry_msgs::PoseStamped ps_start_planner, ps_goal_planner; //Transform poses to planner frame
  tf2::doTransform(ps_start, ps_start_planner, p_T_g);
  tf2::doTransform(ps_goal, ps_goal_planner, p_T_g);

  const Vec2f start(ps_start_planner.pose.position.x, ps_start_planner.pose.position.y);
  const Vec2f goal(ps_goal_planner.pose.position.x, ps_goal_planner.pose.position.y);

  // Run JPS Planner //TODO handle bool returns from "plan" methods properly
  planner_ptr_->plan(start, goal, 1, true); // Plan from start to goal using JPS
  auto path_jps = planner_ptr_->getRawPath();
  nav_msgs::Path nav_path_jps;
  create_path2d(path_jps, nav_path_jps, w_T_p, current_time, ps_goal.pose.position.z);
  paths.push_back(nav_path_jps);

  // Run DMP planner
  dmplanner_ptr_->computePath(start, goal, path_jps); // Compute the path given the jps path
  auto path_dmp = dmplanner_ptr_->getPath();
  nav_msgs::Path nav_path_dmp;
  create_path2d(path_dmp, nav_path_dmp, w_T_p, current_time, ps_goal.pose.position.z);
  paths.push_back(nav_path_dmp);

  if (!paths.empty())
  {
    path_pub_.publish(paths[0]);
    dmp_path_pub_.publish(paths[1]);
    return true;
  }
  else
    return false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plan2d_to_path");

  Plan2DToPath p2dp;

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
