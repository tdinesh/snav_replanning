#include "read_map.hpp"
#include <jps_basis/data_utils.h>
#include <jps_planner/jps_planner/jps_planner.h>
#include <jps_planner/distance_map_planner/distance_map_planner.h>

#include <math.h>
#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <kr_replanning_msgs/VoxelMap.h>
#include <kr_replanning_msgs/SetWindow.h>
#include <std_srvs/SetBool.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

using namespace JPS;

class Plan3DToPath
{
public:
  Plan3DToPath();
private:
  void setUpJPS();
  void setUpDMP();
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void mapCallback(const kr_replanning_msgs::VoxelMap::ConstPtr& msg);
  void waypointsCallback(const nav_msgs::Path::ConstPtr& msg);
  int calculate_id(const Vec3i dim, int x_ind, int y_ind, int z_ind);
  void readMap(std::string map_file);
  void publishMap();
  bool do_planning(geometry_msgs::PoseStamped& ps_start, geometry_msgs::PoseStamped& ps_goal, std::vector<nav_msgs::Path>& paths);
  void create_path(vec_Vec3f planner_path, nav_msgs::Path& path, const geometry_msgs::TransformStamped& transform, const ros::Time current_time);
  bool computePlan(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res);
  ///void createWindowMap(const geometry_msgs::TransformStamped& bl_T_p, const Vec3i& dim, const decimal_t& resolution, std::vector<signed char>& map);
  //bool setWindowMap(kr_replanning_msgs::SetWindow::Request& req, kr_replanning_msgs::SetWindow::Response& res);
  void addFloorCeil(const geometry_msgs::TransformStamped& p_T_b, const geometry_msgs::TransformStamped& b_T_w, const Vec3i& dim, const decimal_t& resolution, std::vector<signed char>& map);


  ros::NodeHandle nh_, pnh_;

  ros::Publisher path_pub_, dmp_path_pub_, map_marker_pub_;
  ros::Subscriber map_sub_, odom_sub_, waypoints_sub_;
  nav_msgs::Odometry odom_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::ServiceServer plan_service_;
  ros::ServiceServer window_service_;

  std::shared_ptr<VoxelMapUtil> map_util_;
  bool map_initialized_;
  bool yaml_map_;
  bool fake_wall_;
  bool floor_ceil_;
  double celing_height_, floor_height_;
  std::string yaml_file_;
  std::string world_frame_, planning_frame_, base_frame_;

  std::vector<Eigen::Vector3f> corners_baselinkf_;

  std::unique_ptr<JPSPlanner3D> planner_ptr_;
  std::unique_ptr<DMPlanner3D> dmplanner_ptr_;

  //mutex for changes of variables
  boost::mutex map_mutex_, odom_mutex_;
};

Plan3DToPath::Plan3DToPath(): tf_listener_(tf_buffer_)
{
  pnh_ = ros::NodeHandle("~");

  pnh_.param("fake_wall", fake_wall_, false);
  pnh_.param("yaml_map", yaml_map_, false);
  pnh_.param<std::string>("yaml_file", yaml_file_, "map.yaml");

  //~ We do planning in the frame of the yaml file or VoxelMap (not octomap)
  //~ For the yaml case, there are 3 frames, world (gazebo), map (octomap), and yaml (written map file, has to be shifted if the octomap has any negative values)
  //~ There is also a base_link for the robot local frame, but it is not needed by the planning (we use the ground_truth/odom topic in the world frame)
  //~ For the VoxelMap taken in by subscription case, there is the world (gazebo) and the map (local frame to the robot created by e.g. SLAM)
  pnh_.param<std::string>("world_frame", world_frame_, "/world");
  pnh_.param<std::string>("base_frame", base_frame_, "base_link");

  pnh_.param("floor_ceiling", floor_ceil_, false);
  pnh_.param("celing_height", celing_height_, 3.0);
  pnh_.param("floor_height", floor_height_, 0.1);

  map_initialized_ = false;

  if(yaml_map_)
  {
    ROS_WARN("Using yaml map %s", yaml_file_.c_str());
    readMap(yaml_file_);
    publishMap();
  }

  map_util_ = std::make_shared<VoxelMapUtil>();

  //#TODO Setup planners, map_utils_ needs to be initialized, maybe init with empty?
  //setUpJPS();
  //setUpDMP();

  path_pub_ = nh_.advertise<nav_msgs::Path>("jps_path", 1);
  dmp_path_pub_ = nh_.advertise<nav_msgs::Path>("dmp_path", 1);
  map_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("map_check", 1);

  if(!yaml_map_)
    map_sub_ = nh_.subscribe<kr_replanning_msgs::VoxelMap>("rb_to_voxel_map", 1, boost::bind(&Plan3DToPath::mapCallback, this, _1));

  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&Plan3DToPath::odomCallback, this, _1));
  waypoints_sub_ = nh_.subscribe<nav_msgs::Path>("waypoints", 2, boost::bind(&Plan3DToPath::waypointsCallback,  this, _1));

  plan_service_ = nh_.advertiseService("jps_plan_service", &Plan3DToPath::computePlan, this);
  //window_service_ = nh_.advertiseService("jps_window_service", &Plan3DToPath::setWindowMap, this);
}

void Plan3DToPath::setUpJPS()
{
  planner_ptr_.reset(new JPSPlanner3D(true)); // Declare a planner
  planner_ptr_->setMapUtil(map_util_); // Set collision checking function
  planner_ptr_->updateMap();
}

void Plan3DToPath::setUpDMP()
{
  dmplanner_ptr_.reset(new DMPlanner3D(true)); // Declare a planner
  dmplanner_ptr_->setMap(map_util_,Vec3f(0.0, 0.0, 0.0)); // Set collision checking function
  //~ dmplanner_ptr->updateMap();
  dmplanner_ptr_->setPotentialRadius(Vec3f(2.5, 2.5, 2.5)); // Set 3D potential field radius
  dmplanner_ptr_->setSearchRadius(Vec3f(1.5, 1.5, 1.5)); // Set the valid search region around given path
}

void Plan3DToPath::readMap(std::string map_file)
{
  // Read the map from yaml
  ROS_INFO("reading map (takes a few seconds)");
  MapReader<Vec3i, Vec3f> reader(map_file.c_str(), true);
  if(!reader.exist())
  {
    ROS_ERROR("Cannot read input file [%s]!",map_file.c_str());
    return;
  }

  // store map in map_util_
  map_util_->setMap(reader.origin(), reader.dim(), reader.data(), reader.resolution());
  planning_frame_ = "yaml";
  map_initialized_ = true;
  ROS_INFO("Map initialized!");
}

void Plan3DToPath::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(odom_mutex_);
  odom_ = *msg;
}

void Plan3DToPath::mapCallback(const kr_replanning_msgs::VoxelMap::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(map_mutex_);

  kr_replanning_msgs::VoxelMap vo_map = *msg;
  Vec3f ori(vo_map.origin.x, vo_map.origin.y, vo_map.origin.z);
  Vec3i dim(vo_map.dim.x, vo_map.dim.y, vo_map.dim.z);
  decimal_t res = vo_map.resolution;
  std::vector<signed char> map = vo_map.data;
  planning_frame_ = vo_map.header.frame_id;

  ros::Time current_time = ros::Time::now();
  if(fake_wall_)
  {
    //Transform from base_link (source frame) to planner frame (target frame)
    geometry_msgs::TransformStamped bl_T_p;
    try
    {
      bl_T_p = tf_buffer_.lookupTransform(planning_frame_, base_frame_, ros::Time(0));
    }
    catch (tf2::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      return;
    }

    /*
    Eigen::Affine3d dT_bl_p;
    tf::transformTFToEigen(transform_bl_to_p, dT_bl_p);
    Eigen::Affine3f Tf_baselink_to_planner = dT_bl_p.cast<float>();
    */

    //createWindowMap(bl_T_p, dim, res, map);
  }

  if(floor_ceil_)
  {
    //Transform from base_link (source frame) to planner frame (target frame)
    geometry_msgs::TransformStamped p_T_b;

    //Transform from world (source frame) to base link frame (target frame)
    geometry_msgs::TransformStamped b_T_w;
    try
    {
      // lookup transform target (frame to which data should be transformed) to source(frame where the data originated )
      p_T_b = tf_buffer_.lookupTransform(planning_frame_, base_frame_, ros::Time(0));
      b_T_w = tf_buffer_.lookupTransform(base_frame_, world_frame_, ros::Time(0));
    }
    catch (tf2::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      return;
    }

    addFloorCeil(p_T_b, b_T_w, dim, res, map);
  }

  map_util_->setMap(ori, dim, map, res);

  if (!map_initialized_)
  {
    map_initialized_ = true;
    ROS_INFO("Map initialized! frame %s", planning_frame_.c_str());
  }
  publishMap();
}


void Plan3DToPath::addFloorCeil(const geometry_msgs::TransformStamped& p_T_b, const geometry_msgs::TransformStamped& b_T_w, const Vec3i& dim, const decimal_t& resolution, std::vector<signed char>& map)
{
  geometry_msgs::Point w_pt_floor, w_pt_ceiling; //Points in world frame
  w_pt_floor.x = 0.0;
  w_pt_floor.y = 0.0;
  w_pt_floor.z = floor_height_;

  w_pt_ceiling.x = 0.0;
  w_pt_ceiling.y = 0.0;
  w_pt_ceiling.z = celing_height_;

  geometry_msgs::Point b_pt_floor, b_pt_ceiling; //Points in base_link frame
  tf2::doTransform(w_pt_floor, b_pt_floor, b_T_w);
  tf2::doTransform(w_pt_ceiling, b_pt_ceiling, b_T_w);

  ROS_WARN_STREAM("bl pts " << b_pt_ceiling  << " " << b_pt_floor);

  //Get floor, celining points directly below and above robot
  b_pt_floor.x = 0.0; b_pt_floor.y = 0.0;
  b_pt_ceiling.x = 0.0; b_pt_ceiling.y = 0.0;

  geometry_msgs::Point p_pt_floor, p_pt_ceiling; //Points in planner frame
  tf2::doTransform(b_pt_floor, p_pt_floor, p_T_b);
  tf2::doTransform(b_pt_ceiling, p_pt_ceiling, p_T_b);

  ROS_WARN_STREAM("planner pts " << p_pt_ceiling  << " " << p_pt_floor);

  // convert position to map indices
  Eigen::Vector3i floor_ind, ceiling_ind;
  floor_ind = map_util_->floatToInt(Vec3f(p_pt_floor.x, p_pt_floor.y, p_pt_floor.z));
  ceiling_ind = map_util_->floatToInt(Vec3f(b_pt_ceiling.x, b_pt_ceiling.y, b_pt_ceiling.z));

  ROS_WARN_STREAM(" " << floor_ind  << " " << ceiling_ind);

  for(int x = 0; x < dim(0); x ++)
  {
    for(int y = 0; y < dim(1); y ++)
    {
      int id = -1;

      id = calculate_id(dim,x, y,floor_ind(2)); //calculate id from map indices
      if (id>=0)
        map[id]=100; //id=-1 if id out of range of map (80x80x80)

      id = calculate_id(dim,x, y,ceiling_ind(2)); //calculate id from map indices
      if (id>=0)
        map[id]=100; //id=-1 if id out of range of map (80x80x80)

    }
  }
}

int Plan3DToPath::calculate_id(const Vec3i dim, int x_ind, int y_ind, int z_ind)
{
  int id = x_ind + dim[0] * y_ind + dim[0] * dim[1] * z_ind;
  if (x_ind < dim[0] && y_ind < dim[1] && z_ind < dim[2] && x_ind >= 0 && y_ind >= 0 && z_ind >= 0)
    return id;
  else
  {
    //~ ROS_ERROR("Octomap data not in dimensions of map");
    return -1;
  }
}

/*
bool Plan3DToPath::setWindowMap(kr_replanning_msgs::SetWindow::Request& req, kr_replanning_msgs::SetWindow::Response& res)
{
  ROS_INFO("set window called");
  boost::mutex::scoped_lock lock(map_mutex_);

  if(!req.fake_wall)
  {
    fake_wall_ = false;
    res.success = true;
    corners_baselinkf_.clear();
    return true;
  }
  else
  {
    std::string frame_id = req.frame_id.data;
    if(frame_id != "left_stereo")
    {
      ROS_ERROR("Please provide corners in left_stereo frame"); //TODO use tf and transform points
      res.success = false;
      return false;
    }
    if(req.window_corners.size()!=4)
    {
      ROS_ERROR("Please provide 4 corners of window");
      res.success = false;
      return false;
    }

    //TODO send corners in world frame from state machine, planner does not know cam frame
    ros::Time current_time = ros::Time::now();
    tf::StampedTransform transform_c_to_bl;
    try
    {
      //listener_.waitForTransform(base_frame_, "left_stereo", current_time, ros::Duration(0.5)); //TODO use frame_id passed from service
      listener_.lookupTransform(base_frame_, "left_stereo", current_time, transform_c_to_bl);
    }
    catch (tf2::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      res.success = false;
      return false;
    }

    Eigen::Affine3d dT_c_bl;
    tf::transformTFToEigen(transform_c_to_bl, dT_c_bl);
    Eigen::Affine3f Tf_camera_to_baself = dT_c_bl.cast<float>();
    corners_baselinkf_.reserve(req.window_corners.size());

    for(const auto& it: req.window_corners)
    {
      Eigen::Vector4f p;
      p[0] = it.x;
      p[1] = it.y;
      p[2] = it.z;
      p[3] = 1;
      p = Tf_camera_to_baself * p;

      corners_baselinkf_.emplace_back(p.head(3));
    }

    fake_wall_ = true;
    res.success = true;
    return true;
  }

  res.success = false;
  return false;
}
*/

/*
void Plan3DToPath::createWindowMap(const geometry_msgs::TransformStamped& bl_T_p, const Vec3i& dim, const decimal_t& resolution, std::vector<signed char>& map)
{

  Eigen::Affine3d dT_bl_p;
  tf::transformTFToEigen(transform_bl_to_p, dT_bl_p);
  Eigen::Affine3f Tf_baselink_to_planner = dT_bl_p.cast<float>();

  //assumes corner order is from top left, clockwise
  Eigen::Vector3i ind;
  Eigen::Vector4f bl_pos, vm_pos;
  bl_pos[3] = 1;

  int max_y_ind = (int)sqrt(dim[0]*dim[0]+dim[1]*dim[1])/2/.9; //half num points in the base_link y frame
  int min_z_ind = -Tf_baselink_to_planner.translation()[0]/resolution; //num points in the base_link z frame below robot
  int max_z_ind = dim[2] + min_z_ind; //num points in the base_link z frame above robot

  for (int i=-max_y_ind; i<max_y_ind; i++)
  { //(y_ind bl) (x and y_ind vm)
    for (int j = min_z_ind; j<max_z_ind; j++)
    { //z_ind (bl and vm)
      for (int k =-1; k<=1; k++)
      { //x_ind (bl)

        bl_pos[1] = i*resolution*.9; // y_ind bl
        bl_pos[2] = j*resolution; //z_ind bl
        bl_pos[0] = corners_baselinkf_[0][0] + k*resolution;

        if (bl_pos[1]>corners_baselinkf_[1][1] && bl_pos[1]<corners_baselinkf_[0][1] && bl_pos[2]<corners_baselinkf_[1][2] && bl_pos[2]>corners_baselinkf_[2][2])
          continue; //take out window

        vm_pos = Tf_baselink_to_planner * bl_pos; //transform from base_link to voxel_map


        ind = map_util_->floatToInt(vm_pos.head(3).cast<double>()); // convert position to map indices
        int id = calculate_id(dim,ind[0],ind[1],ind[2]); //calculate id from map indices
        if (id>=0)
          map[id]=100; //id=-1 if id out of range of map (80x80x80)
      }
    }
  }
}*/

bool Plan3DToPath::computePlan(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res)
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

void Plan3DToPath::waypointsCallback(const nav_msgs::Path::ConstPtr& msg)
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
    ROS_ERROR("No map initialized");

  //#TODO does the planner needs to be reset everytime?
  setUpJPS();
  setUpDMP();

  bool ret = do_planning(ps_start, ps_goal, paths);
  }
}

void Plan3DToPath::publishMap()
{
  const Vec3i dim = map_util_->getDim();
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

  marker.points.reserve(dim(0)*dim(1)*dim(2));

  for(int x = 0; x < dim(0); x ++) {
    for(int y = 0; y < dim(1); y ++) {
      for(int z = 0; z < dim(2); z ++) {
        if(!map_util_->isFree(Vec3i(x, y,z))) {
        Vec3f pt = map_util_->intToFloat(Vec3i(x, y,z));
        geometry_msgs::Point point;
        point.x = pt(0);
        point.y = pt(1);
        point.z = pt(2);
        marker.points.emplace_back(point);
  }}}}

  map_marker_pub_.publish(marker);
}

void Plan3DToPath::create_path(vec_Vec3f planner_path, nav_msgs::Path& path, const geometry_msgs::TransformStamped& w_T_p, const ros::Time current_time)
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
    path_ps_planner.pose.position.z = it.transpose()[2];

    tf2::doTransform(path_ps_planner, path_ps_world, w_T_p); //Transform to world frame

    // linear interpolation to add points to path for ewok to run better
    if (path.poses.size() > 0)
    {
      float diff_x = path_ps_world.pose.position.x - path.poses.back().pose.position.x;
      float diff_y = path_ps_world.pose.position.y - path.poses.back().pose.position.y;
      float diff_z = path_ps_world.pose.position.z - path.poses.back().pose.position.z;
      int num_steps = int(sqrt(diff_x*diff_x+diff_y*diff_y+diff_z*diff_z)/0.2);
      geometry_msgs::PoseStamped interm_pt = path_ps_world;
      geometry_msgs::PoseStamped last_pt = path.poses.back();
      for (int i = 1; i < num_steps; i++)
      {
        interm_pt.pose.position.x = diff_x*i/num_steps + last_pt.pose.position.x;
        interm_pt.pose.position.y = diff_y*i/num_steps + last_pt.pose.position.y;
        interm_pt.pose.position.z = diff_z*i/num_steps + last_pt.pose.position.z;
        path.poses.push_back(interm_pt);
      }
    }
    path.poses.push_back(path_ps_world);
  }
}

bool Plan3DToPath::do_planning(geometry_msgs::PoseStamped& ps_start, geometry_msgs::PoseStamped& ps_goal, std::vector<nav_msgs::Path>& paths)
{
  /*
  if (ps_goal.pose.position.x == ps_start.pose.position.x && ps_goal.pose.position.y == ps_start.pose.position.y && ps_goal.pose.position.z == ps_start.pose.position.z)
  {
    ROS_ERROR("Start and goal poses are the same. Have you published waypoints?");
    return;
  }*/
  ROS_INFO("Running planner");
  geometry_msgs::TransformStamped w_T_p, p_T_g;
  ros::Time current_time = ros::Time::now();
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
  tf2::doTransform(ps_start_planner, ps_start, p_T_g);
  tf2::doTransform(ps_goal_planner, ps_goal, p_T_g);

  const Vec3f start(ps_start_planner.pose.position.x,ps_start_planner.pose.position.y,ps_start_planner.pose.position.z);
  const Vec3f goal(ps_goal_planner.pose.position.x,ps_goal_planner.pose.position.y,ps_goal_planner.pose.position.z);

  // Run JPS Planner //TODO handle bool returns from "plan" methods properly
  planner_ptr_->plan(start, goal, 1, true); // Plan from start to goal using JPS
  auto path_jps = planner_ptr_->getRawPath();
  nav_msgs::Path nav_path_jps;
  create_path(path_jps, nav_path_jps, w_T_p, current_time);
  paths.push_back(nav_path_jps);

  // Run DMP planner
  dmplanner_ptr_->computePath(start, goal, path_jps); // Compute the path given the jps path
  auto path_dmp = dmplanner_ptr_->getPath();
  nav_msgs::Path nav_path_dmp;
  create_path(path_dmp, nav_path_dmp, w_T_p, current_time);
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
  ros::init(argc, argv, "plan3d_to_path");

  Plan3DToPath p3dp;

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
