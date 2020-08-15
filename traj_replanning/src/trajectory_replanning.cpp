/**
* This file is part of Ewok.
*
* Copyright 2017 Vladyslav Usenko, Technical University of Munich.
* Developed by Vladyslav Usenko <vlad dot usenko at tum dot de>,
* for more information see <http://vision.in.tum.de/research/robotvision/replanning>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Ewok is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Ewok is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Ewok. If not, see <http://www.gnu.org/licenses/>.
*/

#include <thread>
#include <chrono>
#include <map>

#define _USE_MATH_DEFINES // for C++
#include <cmath>

#include <Eigen/StdVector>
#include <Eigen/Core>
//#include <mav_msgs/conversions.h>
//#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <geometry_msgs/Point.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

#include <ewok/polynomial_3d_optimization.h>
#include <ewok/uniform_bspline_3d_optimization.h>

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <kr_mav_manager/Vec4.h>
#include <kr_tracker_msgs/TrajectoryTrackerAction.h>
#include <kr_tracker_msgs/Transition.h>
#include <kr_tracker_msgs/TrackerStatus.h>
#include <kr_replanning_msgs/TrackPathAction.h>
#include <kr_replanning_msgs/ConfigureReplanning.h>

#include <angles/angles.h>

const int POW = 6;

static const std::string traj_tracker_str("kr_trackers/TrajectoryTracker");

class KrTraj
{
public:
  KrTraj();
  ~KrTraj();
  void sendCommand();
private:

  void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg);
  void pathCallback(const nav_msgs::Path::ConstPtr& msg);
  void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void tracker_done_callback(const actionlib::SimpleClientGoalState& state, const kr_tracker_msgs::TrajectoryTrackerResultConstPtr& result);
  void downPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void track_path_callback();
  void preempt_callback();
  void processPath(const nav_msgs::Path::ConstPtr& msg);
  void setTrackingPath(const Eigen::Vector4d& limits, const geometry_msgs::Point& start, ewok::PolynomialTrajectory3D<10>::Ptr& traj);
  bool getLookAheadCosts(const geometry_msgs::Point& lookahead, const Eigen::Affine3f& bl_w_transform,
    const Eigen::Vector3d& normal_to_traj, geometry_msgs::PoseStamped& min_cost_pt);
  bool getJpsTraj(const double& traj_time, const Eigen::Affine3f& bl_w_transform, geometry_msgs::PoseStamped& min_cost_pt);
  void updateTrackingPath(const Eigen::Vector4d& limits, const geometry_msgs::Point& start, ewok::PolynomialTrajectory3D<10>::Ptr& traj);
  bool configureParams(kr_replanning_msgs::ConfigureReplanning::Request& req, kr_replanning_msgs::ConfigureReplanning::Response& res);
  void edrbMoveVolume(Eigen::Vector3f& origin);

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber path_sub_;

  message_filters::Subscriber<sensor_msgs::Image> depth_image_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub_, down_points_sub_;

  ros::ServiceClient service_client_;
  ros::ServiceClient jps_service_client_;

  ros::ServiceClient srv_tracker_transition_;

  ros::ServiceServer configure_service_;

  typedef actionlib::SimpleActionClient<kr_tracker_msgs::TrajectoryTrackerAction> TrajectoryClientType;
  TrajectoryClientType traj_tracker_client_;

  ros::Publisher occ_marker_pub_, free_marker_pub_, dist_marker_pub_, trajectory_pub_, current_traj_pub_, global_traj_marker_pub_, lookahead_costs_pub_;
  std::unique_ptr<tf::TransformListener> listener_;
  std::unique_ptr<tf::TransformListener> listener_down_;

  std::unique_ptr<tf::MessageFilter<sensor_msgs::Image> > tf_filter_im_;
  std::unique_ptr<tf::MessageFilter<sensor_msgs::PointCloud2> > tf_filter_fc_;
  std::unique_ptr<tf::MessageFilter<sensor_msgs::PointCloud2> > tf_filter_dp_;

  double dt_;
  int num_opt_points_;

  bool edrb_initialized_, path_initialized_, path_tracking_;
  bool turn_in_place_;

  std::ofstream f_time, opt_time;

  ewok::PolynomialTrajectory3D<10>::Ptr global_traj_;
  ewok::PolynomialTrajectory3D<10>::Ptr local_traj_;
  ewok::PolynomialTrajectory3D<10>::Ptr orig_global_traj_;
  ewok::EuclideanDistanceRingBuffer<POW>::Ptr edrb_;
  ewok::UniformBSpline3DOptimization<6>::Ptr spline_optimization_;

  double max_velocity_, max_acceleration_;
  double distance_threshold_;
  double local_time_;
  double traj_reset_time_;
  double last_yaw_;
  double inflation_size_;
  double resolution_;
  double min_yaw_ang_diff_;
  double initial_traj_yaw_;
  bool once_;
  bool jps_always_;
  double edrb_size_;
  bool inflate_;

  double lookahead_time_;
  double lookahead_grid_size_width_;
  double lookahead_grid_size_height_;
  double lookahead_grid_resolution_;
  double lookahead_gain_traj_dist_;
  double lookahead_gain_obstacle_dist_;
  double lookahead_gain_ray_;
  ros::Time last_time_force_jps_;
  ros::Time last_edrb_move_volume_;

  bool disable_obstacle_avoid_;
  bool disable_jps_;

  Eigen::Vector3d last_cmd_;

  std::string map_frame_, odom_frame_, base_frame_;
  bool use_current_start_, use_goto_;

  typedef actionlib::SimpleActionServer<kr_replanning_msgs::TrackPathAction> ServerType;
  std::shared_ptr<ServerType> tracker_server_;

};

KrTraj::KrTraj():
  traj_tracker_client_(nh_, "trackers_manager/trajectory_tracker/TrajectoryTracker", true)
{
  pnh_ = ros::NodeHandle("~");

  edrb_initialized_ = false;
  path_initialized_ = false;
  path_tracking_ = false;
  local_time_ = 0.0;
  traj_reset_time_ = 0.0;
  last_yaw_ = 0.0;
  last_cmd_.setZero();
  turn_in_place_ = false;
  initial_traj_yaw_ = 0.0;
  once_ = false;

  last_time_force_jps_ = ros::Time::now();
  last_edrb_move_volume_ = ros::Time::now();

  pnh_.param("max_velocity", max_velocity_, 0.5);
  pnh_.param("max_acceleration", max_acceleration_, 0.6);

  Eigen::Vector4d limits(max_velocity_, max_acceleration_, 0, 0);

  pnh_.param("resolution", resolution_, 0.1);
  pnh_.param("size", edrb_size_, 2.0);

  edrb_.reset(new ewok::EuclideanDistanceRingBuffer<POW>(resolution_, edrb_size_));

  pnh_.param("distance_threshold", distance_threshold_, 0.1);

  pnh_.param("dt", dt_, 0.5);
  pnh_.param("num_opt_points", num_opt_points_, 7);

  bool use_points = true;
  pnh_.param("use_points", use_points, true);

  pnh_.param<std::string>("map_frame", map_frame_, "world");
  pnh_.param<std::string>("base_frame", base_frame_, "base_link");

  pnh_.param("use_current_start", use_current_start_, true);
  pnh_.param("use_goto", use_goto_, true);

  ROS_INFO("dt_: %f, num_opt_points_: %d", dt_, num_opt_points_);

  bool use_down_cam = false;
  pnh_.param("use_down_cam", use_down_cam, false);
  pnh_.param("inflation_size", inflation_size_, 0.5);
  inflation_size_ = inflation_size_/2.0;

  pnh_.param("min_yaw_ang_diff", min_yaw_ang_diff_, 10.0);

  pnh_.param("disable_obstacle_avoid", disable_obstacle_avoid_, false);
  pnh_.param("disable_jps", disable_jps_, false);
  pnh_.param("jps_always", jps_always_, true);
  pnh_.param("inflate_ring_buffer", inflate_, false);

  pnh_.param("lookahead_time", lookahead_time_, 3.0);

  pnh_.param("lookahead_grid_size_width", lookahead_grid_size_width_, 3.5);
  pnh_.param("lookahead_grid_size_height", lookahead_grid_size_height_, 3.5);
  pnh_.param("lookahead_grid_resolution", lookahead_grid_resolution_, 0.3);
  pnh_.param("lookahead_gain_traj_dist", lookahead_gain_traj_dist_, 1.0);
  pnh_.param("lookahead_gain_obstacle_dist", lookahead_gain_obstacle_dist_, 2.0);
  pnh_.param("lookahead_gain_ray", lookahead_gain_ray_, 1.0);

  min_yaw_ang_diff_ = min_yaw_ang_diff_*3.142/180.0;

  listener_.reset(new tf::TransformListener());
  listener_down_.reset(new tf::TransformListener());

  occ_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("ring_buffer/occupied", 5);
  free_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("ring_buffer/free", 5);
  dist_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("ring_buffer/distance", 5);
  lookahead_costs_pub_ = nh_.advertise<visualization_msgs::Marker>("lookahead_cost", 5);

  trajectory_pub_ = nh_.advertise<geometry_msgs::Point>("command/point", 10);

  global_traj_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("global_trajectory", 1, true);

  current_traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("optimal_trajectory", 1, true);

  service_client_ = nh_.serviceClient<kr_mav_manager::Vec4>("mav_services/goTo");

  jps_service_client_ = nh_.serviceClient<nav_msgs::GetPlan>("jps_plan_service");

  if(use_points){
    points_sub_.subscribe(nh_, "input_point_cloud", 3);
    tf_filter_fc_.reset(new tf::MessageFilter<sensor_msgs::PointCloud2>(points_sub_, *listener_, map_frame_, 3));
    tf_filter_fc_->registerCallback(boost::bind(&KrTraj::pointsCallback, this, _1));
  }
  else{
    depth_image_sub_.subscribe(nh_, "depth_image", 3);
    tf_filter_im_.reset(new tf::MessageFilter<sensor_msgs::Image>(depth_image_sub_, *listener_, map_frame_, 3));
    tf_filter_im_->registerCallback(boost::bind(&KrTraj::depthImageCallback, this, _1));
  }

  if(use_down_cam){
    down_points_sub_.subscribe(nh_, "down_cam_points", 3);
    //down_points_sub_ = nh_.subscribe(nh_, "down_cam_points", 3, boost::bind(&KrTraj::downPointsCallback, this, _1));
    tf_filter_dp_.reset(new tf::MessageFilter<sensor_msgs::PointCloud2>(down_points_sub_, *listener_down_, map_frame_, 3));
    tf_filter_dp_->registerCallback(boost::bind(&KrTraj::downPointsCallback, this, _1));
  }

  path_sub_  = nh_.subscribe<nav_msgs::Path>("waypoints", 5,  boost::bind(&KrTraj::pathCallback, this, _1));

  // Set up the action server.
  tracker_server_ = std::shared_ptr<ServerType>(new ServerType(nh_, "TrackPathAction", false));
  tracker_server_->registerGoalCallback(boost::bind(&KrTraj::track_path_callback, this));
  tracker_server_->registerPreemptCallback(boost::bind(&KrTraj::preempt_callback, this));

  tracker_server_->start();

  srv_tracker_transition_ = nh_.serviceClient<kr_tracker_msgs::Transition>("trackers_manager/transition");

  if (!traj_tracker_client_.waitForServer(ros::Duration(3.0))) {
    ROS_ERROR("TrajectoryTracker server not found.");
  }

  configure_service_ = nh_.advertiseService("configure_service", &KrTraj::configureParams, this);

  ROS_WARN("Traj replanning initialized");
}

KrTraj::~KrTraj()
{
}

bool KrTraj::configureParams(kr_replanning_msgs::ConfigureReplanning::Request& req, kr_replanning_msgs::ConfigureReplanning::Response& res)
{
  disable_jps_ = req.disable_jps;
  disable_obstacle_avoid_ = req.disable_obstacle_avoid;
  inflate_ = req.inflate_ring_buffer;

  ROS_WARN("Reconfigure callback jps %d obstacle_avoid %d inflate %d", disable_jps_, disable_obstacle_avoid_, inflate_);

  if(req.reset_ring_buffer)
  {
    edrb_.reset(new ewok::EuclideanDistanceRingBuffer<POW>(resolution_, edrb_size_));
    if(spline_optimization_)
      spline_optimization_->setDistanceBuffer(edrb_);
  }
  res.success = true;
  return true;
}

void KrTraj::depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  const float fx = 213.57090759277344;
  const float fy = 213.57090759277344;
  const float cx = 167.15345764160156;
  const float cy = 118.86390686035156;

  tf::StampedTransform transform;
  try
  {
    listener_->lookupTransform(map_frame_, msg->header.frame_id, msg->header.stamp, transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_INFO("Couldn't get transform");
    ROS_WARN("%s",ex.what());
    return;
  }

  Eigen::Affine3d dT_w_c;
  tf::transformTFToEigen(transform, dT_w_c);

  Eigen::Affine3f T_w_c = dT_w_c.cast<float>();

  float * data = (float *) cv_ptr->image.data;

  auto t1 = std::chrono::high_resolution_clock::now();

  ewok::EuclideanDistanceRingBuffer<POW>::PointCloud cloud1;

  for(int u=0; u < cv_ptr->image.cols; u+=4)
  {
    for(int v=0; v < cv_ptr->image.rows; v+=4)
    {
      float val = data[v*cv_ptr->image.cols + u];

      if(std::isfinite(val))
      {
        Eigen::Vector4f p;
        p[0] = val*(u - cx)/fx;
        p[1] = val*(v - cy)/fy;
        p[2] = val;
        p[3] = 1;

        p = T_w_c * p;

        cloud1.push_back(p);
      }
    }
  }

  Eigen::Vector3f origin = (T_w_c * Eigen::Vector4f(0,0,0,1)).head<3>();

  auto t2 = std::chrono::high_resolution_clock::now();

  if(!edrb_initialized_)
  {
    Eigen::Vector3i idx;
    edrb_->getIdx(origin, idx);

    ROS_INFO_STREAM("Origin: " << origin.transpose() << " idx " << idx.transpose());

    edrb_->setOffset(idx);

    edrb_initialized_ = true;
  }
  else
  {
    Eigen::Vector3i origin_idx, offset, diff;
    edrb_->getIdx(origin, origin_idx);

    offset = edrb_->getVolumeCenter();
    diff = origin_idx - offset;

    while(diff.array().any())
    {
      edrb_->moveVolume(diff);

      offset = edrb_->getVolumeCenter();
      diff = origin_idx - offset;
    }
  }
  auto t3 = std::chrono::high_resolution_clock::now();

  edrb_->insertPointCloud(cloud1, origin);

  auto t4 = std::chrono::high_resolution_clock::now();

  f_time << std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count() << " " <<
            std::chrono::duration_cast<std::chrono::nanoseconds>(t3-t2).count() << " " <<
            std::chrono::duration_cast<std::chrono::nanoseconds>(t4-t3).count() << std::endl;

  visualization_msgs::Marker m_occ, m_free;
  edrb_->getMarkerOccupied(m_occ);
  edrb_->getMarkerFree(m_free);

  m_occ.header.stamp = msg->header.stamp;
  m_free.header.stamp = msg->header.stamp;
  m_occ.header.frame_id = map_frame_;
  m_free.header.frame_id = map_frame_;
  occ_marker_pub_.publish(m_occ);
  free_marker_pub_.publish(m_free);
}

void KrTraj::downPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{

  if(msg->header.frame_id != map_frame_)
  {
    ROS_ERROR("VIO points frame %s does not match %s frame",msg->header.frame_id.c_str(), map_frame_.c_str());
    return;
  }

  tf::StampedTransform transform;
  try
  {
    listener_down_->lookupTransform(map_frame_, "left_stereo", msg->header.stamp, transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_INFO("Couldn't get transform");
    ROS_WARN("%s",ex.what());
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg((*msg), pcl_cloud);

  Eigen::Affine3d dT_w_c;
  tf::transformTFToEigen(transform, dT_w_c);

  Eigen::Affine3f T_w_c = dT_w_c.cast<float>();

  auto t1 = std::chrono::high_resolution_clock::now();

  ewok::EuclideanDistanceRingBuffer<POW>::PointCloud cloud1;

  int inflation_step = inflation_size_/resolution_;
  float inflate_size = inflation_size_/2.0;

  for(int i=0; i < pcl_cloud.points.size(); i++)
  {
    pcl::PointXYZ pt = pcl_cloud.points[i];

    Eigen::Vector4f p;
    p[0] = pt.x;
    p[1] = pt.y;
    p[2] = pt.z;
    p[3] = 1; //#TODO add additional points in same X plane nearby sparce points?

    cloud1.push_back(p);

    float min_x = pt.x - inflate_size;
    float min_y = pt.y - inflate_size;
    for(int j=0; j<inflation_step ; j++)
    {
      for(int k=0; k<inflation_step; k++)
      {
        p[0] = min_x + j*resolution_;
        p[1] = min_y + k*resolution_;
        cloud1.push_back(p);
      }
    }
    //p = T_w_c * p;

  }

  Eigen::Vector3f origin = (T_w_c * Eigen::Vector4f(0,0,0,1)).head<3>();

  auto t2 = std::chrono::high_resolution_clock::now();

  if(!edrb_initialized_)
  {
    Eigen::Vector3i idx;
    edrb_->getIdx(origin, idx);

    ROS_INFO_STREAM("Origin: " << origin.transpose() << " idx " << idx.transpose());

    edrb_->setOffset(idx);

    edrb_initialized_ = true;
  }
  else
  {
    Eigen::Vector3i origin_idx, offset, diff;
    edrb_->getIdx(origin, origin_idx);

    offset = edrb_->getVolumeCenter();
    diff = origin_idx - offset;

    while(diff.array().any()) {
      //ROS_INFO("Moving Volume");
      edrb_->moveVolume(diff);

      offset = edrb_->getVolumeCenter();
      diff = origin_idx - offset;
    }
  }

  //ROS_INFO_STREAM("cloud1 size: " << cloud1.size());

  auto t3 = std::chrono::high_resolution_clock::now();
  edrb_->insertPointCloud(cloud1, origin);
  auto t4 = std::chrono::high_resolution_clock::now();

  f_time << std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count() << " " <<
            std::chrono::duration_cast<std::chrono::nanoseconds>(t3-t2).count() << " " <<
            std::chrono::duration_cast<std::chrono::nanoseconds>(t4-t3).count() << std::endl;

  visualization_msgs::Marker m_occ, m_free;
  edrb_->getMarkerOccupied(m_occ);
  edrb_->getMarkerFree(m_free);

  m_occ.header.stamp = msg->header.stamp;
  m_free.header.stamp = msg->header.stamp;
  m_occ.header.frame_id = map_frame_;
  m_free.header.frame_id = map_frame_;
  occ_marker_pub_.publish(m_occ);
  free_marker_pub_.publish(m_free);
}

void KrTraj::edrbMoveVolume(Eigen::Vector3f& origin)
{
  if(!edrb_initialized_)
  {
    Eigen::Vector3i idx;
    edrb_->getIdx(origin, idx);
    edrb_->setOffset(idx);
    edrb_initialized_ = true;
    ROS_INFO_STREAM("Origin: " << origin.transpose() << " idx " << idx.transpose());
  }
  else
  {
    Eigen::Vector3i origin_idx, offset, diff;
    edrb_->getIdx(origin, origin_idx);
    offset = edrb_->getVolumeCenter();
    diff = origin_idx - offset;
    while(diff.array().any())
    {
      //ROS_INFO("Moving Volume");
      edrb_->moveVolume(diff);

      offset = edrb_->getVolumeCenter();
      diff = origin_idx - offset;
    }
  }
  last_edrb_move_volume_ = ros::Time::now();
}

void KrTraj::pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  if(disable_obstacle_avoid_)
    return;

  tf::StampedTransform transform;
  try
  {
    listener_->lookupTransform(map_frame_, msg->header.frame_id, msg->header.stamp, transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_INFO("Couldn't get transform");
    ROS_WARN("%s",ex.what());
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg((*msg), pcl_cloud);

  Eigen::Affine3d dT_w_c;
  tf::transformTFToEigen(transform, dT_w_c);

  Eigen::Affine3f T_w_c = dT_w_c.cast<float>();

  auto t1 = std::chrono::high_resolution_clock::now();

  ewok::EuclideanDistanceRingBuffer<POW>::PointCloud cloud1;

  int infl_rad[3] = {1,1,1};
  for(int i=0; i < pcl_cloud.points.size(); i++)
  {
    pcl::PointXYZ pt = pcl_cloud.points[i];

    Eigen::Vector4f p;
    p[0] = pt.x;
    p[1] = pt.y;
    p[2] = pt.z;
    p[3] = 1;

    p = T_w_c * p;
    cloud1.push_back(p); //TODO push vec3f in cloud?

    Eigen::Vector4f p_inf;
    p_inf[3] = 1;
    float x_w = p[0];
    float y_w = p[1];
    float z_w = p[2];

    if(inflate_)
    {
      for (int l=-infl_rad[0]; l<=infl_rad[0];l++)
      {
        for (int m=-infl_rad[1]; m<=infl_rad[1];m++)
        {
          for (int n=-infl_rad[2]; n<=infl_rad[2];n++)
          {
            p_inf[0] = x_w + l*resolution_;
            p_inf[1] = y_w + m*resolution_;
            p_inf[2] = z_w + n*resolution_;
            cloud1.push_back(p_inf);
          }
        }
      }
    }

    cloud1.push_back(p);
  }

  auto t2 = std::chrono::high_resolution_clock::now();
  Eigen::Vector3f origin = (T_w_c * Eigen::Vector4f(0,0,0,1)).head<3>();

  edrbMoveVolume(origin);

  /*
  if(!edrb_initialized_)
  {
    Eigen::Vector3i idx;
    edrb_->getIdx(origin, idx);
    edrb_->setOffset(idx);
    edrb_initialized_ = true;
    ROS_INFO_STREAM("Origin: " << origin.transpose() << " idx " << idx.transpose());
  }
  else
  {
    Eigen::Vector3i origin_idx, offset, diff;
    edrb_->getIdx(origin, origin_idx);
    offset = edrb_->getVolumeCenter();
    diff = origin_idx - offset;
    while(diff.array().any())
    {
      //ROS_INFO("Moving Volume");
      edrb_->moveVolume(diff);

      offset = edrb_->getVolumeCenter();
      diff = origin_idx - offset;
    }
  }
  */

  //ROS_INFO_STREAM("cloud1 size: " << cloud1.size());

  auto t3 = std::chrono::high_resolution_clock::now();
  edrb_->insertPointCloud(cloud1, origin);
  auto t4 = std::chrono::high_resolution_clock::now();

  f_time << std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count() << " " <<
            std::chrono::duration_cast<std::chrono::nanoseconds>(t3-t2).count() << " " <<
            std::chrono::duration_cast<std::chrono::nanoseconds>(t4-t3).count() << std::endl;

  visualization_msgs::Marker m_occ, m_free;
  edrb_->getMarkerOccupied(m_occ);
  edrb_->getMarkerFree(m_free);

  m_occ.header.stamp = msg->header.stamp;
  m_free.header.stamp = msg->header.stamp;
  m_occ.header.frame_id = map_frame_;
  m_free.header.frame_id = map_frame_;
  occ_marker_pub_.publish(m_occ);
  free_marker_pub_.publish(m_free);
}

void KrTraj::tracker_done_callback(const actionlib::SimpleClientGoalState& state, const kr_tracker_msgs::TrajectoryTrackerResultConstPtr& result) {
  ROS_INFO("Goal finished tot_time %2.2f tot_dist %2.2f", result->total_time, result->total_distance_travelled);
}

bool KrTraj::getLookAheadCosts(const geometry_msgs::Point& lookahead_wf, const Eigen::Affine3f& bl_w_transform,
  const Eigen::Vector3d& normal_to_traj, geometry_msgs::PoseStamped& min_cost_pt)
{
  bool found_min_cost = false;

  int num_grid_pts_w = lookahead_grid_size_width_/lookahead_grid_resolution_/2.0;
  int num_grid_pts_h = lookahead_grid_size_height_/lookahead_grid_resolution_/2.0;

  float cost, traj_dist, obstacle_dist;
  float ray_cost;
  visualization_msgs::Marker debug_marker;
  debug_marker.scale.x = .05;
  debug_marker.scale.y = .05;
  debug_marker.scale.z = .05;
  debug_marker.color.a = 1;
  debug_marker.color.r = 1;
  debug_marker.pose.orientation.w = 1.0;
  debug_marker.header.frame_id = map_frame_;
  debug_marker.header.stamp = ros::Time::now();
  debug_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  std_msgs::ColorRGBA color;
  color.a = 1;

  Eigen::Vector3f current_position;
  current_position[0] = bl_w_transform.translation()[0];
  current_position[1] = bl_w_transform.translation()[1];
  current_position[2] = bl_w_transform.translation()[2];

  Eigen::Vector3f n_to_traj = normal_to_traj.cast<float>();
  Eigen::Vector3f grad;
  Eigen::Vector3f pt_wf, pt_temp;
  float obst_dist;
  geometry_msgs::Point gm_pt_wf;
  float min_cost = 10000000;

  //#TODO match lookahed size to fov of camera at lookahed point
  for (int i=-(num_grid_pts_h - 2); i<=num_grid_pts_h; i++) //Avoid too many frontiers below optimal traj
  {
    pt_temp[0] = lookahead_wf.x;
    pt_temp[1] = lookahead_wf.y;
    pt_temp[2] = lookahead_wf.z + i*lookahead_grid_resolution_; //Increent z of temp point

    for (int j=-num_grid_pts_w; j<=num_grid_pts_w; j++)
    {
      pt_wf = pt_temp + (j*lookahead_grid_resolution_)*n_to_traj; //go along the direction of normal to traj

      ray_cost = 0;
      Eigen::Vector3f ray_dir;
      ray_dir = pt_wf - current_position;
      float ray_len = ray_dir.norm();
      ray_dir.normalize();
      int num_steps = int(ray_len/0.2); //Check at 0.3 m resolution along ray
      bool ray_occupied = false;
      int ray_occupied_cost = 100000;
      for (int ii = 1; ii < num_steps; ii++)
      {
        Eigen::Vector3f la = current_position + ii*0.2*ray_dir;
        ray_occupied = edrb_->isOccupied(la);

        gm_pt_wf.x = la[0];
        gm_pt_wf.y = la[1];
        gm_pt_wf.z = la[2];
        color.g = 0.7;
        color.r = 0;
        color.b = 0.7;

        if(ray_occupied)
        {
          color.g = 0;
          color.r = 1;
          color.b = 0;
          debug_marker.colors.push_back(color);
          debug_marker.points.push_back(gm_pt_wf);
          ray_cost = ray_occupied_cost;
          break;
        }
        debug_marker.colors.push_back(color);
        debug_marker.points.push_back(gm_pt_wf);
      }

      obstacle_dist = -edrb_->getDistanceWithGrad(pt_wf,grad) + 3; // whats 3?
      traj_dist = (pow(pt_wf[0] - lookahead_wf.x,2) + pow(pt_wf[1] - lookahead_wf.y,2) + pow(pt_wf[2] - lookahead_wf.z,2)); //TODO make a lookup table, constant for each grid

      cost = lookahead_gain_traj_dist_*traj_dist + lookahead_gain_obstacle_dist_*obstacle_dist + ray_cost;
      color.r = cost/(3*lookahead_gain_obstacle_dist_ + 10*lookahead_gain_traj_dist_);
      color.b  = 1-color.r;
      color.g = 0;

      if (ray_occupied){
         color.g = 0;
         color.r = 1;
         color.b = 0.5;
         //ROS_INFO("lookahead pt cost %f", cost);
       }
      //ROS_INFO("traj_dist %f %f obstacle_dist %f %f ray_cost %f",traj_dist, lookahead_gain_traj_dist_*traj_dist, obstacle_dist, lookahead_gain_obstacle_dist_*obstacle_dist, ray_cost);
      //ROS_INFO("cost %f grid pt %f %f %f", cost, pt_wf[0], pt_wf[1],pt_wf[2]);

      debug_marker.colors.push_back(color);
      gm_pt_wf.x = pt_wf[0];
      gm_pt_wf.y = pt_wf[1];
      gm_pt_wf.z = pt_wf[2];
      debug_marker.points.push_back(gm_pt_wf);

      if (cost<min_cost && !ray_occupied)
      {
        min_cost = cost;
        min_cost_pt.pose.position.x = pt_wf[0];
        min_cost_pt.pose.position.y = pt_wf[1];
        min_cost_pt.pose.position.z = pt_wf[2];
        found_min_cost = true;
      }
    }
  }

  if(found_min_cost)
  {
    //Point with lowest cost
    color.g = 1;
    color.r = 0;
    color.b = 0;
    debug_marker.colors.push_back(color);
    gm_pt_wf.x = min_cost_pt.pose.position.x;
    gm_pt_wf.y = min_cost_pt.pose.position.y;
    gm_pt_wf.z = min_cost_pt.pose.position.z;
    debug_marker.points.push_back(gm_pt_wf);
  }

  lookahead_costs_pub_.publish(debug_marker);
  min_cost_pt.header.stamp = ros::Time::now();
  min_cost_pt.pose.orientation.w =1;
  min_cost_pt.header.frame_id = map_frame_;

  return found_min_cost;
}

bool KrTraj::getJpsTraj(const double& traj_time, const Eigen::Affine3f& bl_w_transform, geometry_msgs::PoseStamped& min_cost_pt)
{
  nav_msgs::GetPlan jps_srv;
  jps_srv.request.start = min_cost_pt;
  jps_srv.request.start.pose.position.x = bl_w_transform.translation()[0];
  jps_srv.request.start.pose.position.y = bl_w_transform.translation()[1];
  jps_srv.request.start.pose.position.z = bl_w_transform.translation()[2];
  jps_srv.request.goal = min_cost_pt;
  if (jps_service_client_.call(jps_srv))
  {
    ROS_INFO("Successful jps service call");

    /*for(int i = 0; i < jps_srv.response.plan.poses.size(); i++)
    {
      geometry_msgs::PoseStamped ps = jps_srv.response.plan.poses[i];
      path.push_back(Eigen::Vector3d(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z));
    }*/

    if(jps_srv.response.plan.poses.size()==0)
    {
      ROS_ERROR("JPS did not return a path");
      return false;
    }

    double lookah_join_time = traj_time + lookahead_time_*2.5;
    Eigen::Vector3d lookah_join = orig_global_traj_->evaluate(lookah_join_time);
    double traj_lt, lookah_join_lt;
    size_t traj_seg_num, lookah_join_seg_num;
    orig_global_traj_->findSegmentNumAndLocalTime(traj_time - 2.0, traj_lt, traj_seg_num);
    orig_global_traj_->findSegmentNumAndLocalTime(lookah_join_time, lookah_join_lt, lookah_join_seg_num);

    //ROS_WARN("l seg %zu tim %g join seg %zu tim %g", lookah_seg_num, lookah_lt, lookah_join_seg_num, lookah_join_lt);

    visualization_msgs::MarkerArray local_traj;
    local_traj.markers.resize(orig_global_traj_->numSegments()+1);
    visualization_msgs::Marker &traj_marker = local_traj.markers[0];
    traj_marker.header.frame_id = map_frame_;
    traj_marker.ns = "gt";
    traj_marker.id = 0;
    traj_marker.type = visualization_msgs::Marker::POINTS;
    traj_marker.action = visualization_msgs::Marker::MODIFY;
    traj_marker.scale.x = 0.01;
    traj_marker.scale.y = 0.01;
    traj_marker.scale.z = 0.01;
    traj_marker.color.a = 1.0;
    traj_marker.lifetime = ros::Duration(0);
    traj_marker.color.r = 1;
    traj_marker.color.g = 0;
    traj_marker.color.b = 1;

    //local_traj.markers[0].points.reserve(jps_srv.response.plan.poses.size());
    for(int i = 0; i < jps_srv.response.plan.poses.size(); i++)
    {
      geometry_msgs::PoseStamped ps = jps_srv.response.plan.poses[i];
      //path.push_back(Eigen::Vector3d(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z));
      local_traj.markers[0].points.push_back(ps.pose.position);
    }

    //Add connecting path from jps to lookah_join point
    geometry_msgs::Point jps_goal = local_traj.markers[0].points.back();
    geometry_msgs::Point jps_start = local_traj.markers[0].points.front();

    std::vector<geometry_msgs::Point>::iterator it = local_traj.markers[0].points.begin();
    if(local_traj.markers[0].points.size() > 2){
      it += 1;
      jps_start = *it; //Take a point slightly front to avoid going back
    }

    float diff_x = lookah_join[0] - jps_goal.x;
    float diff_y = lookah_join[1] - jps_goal.y;
    float diff_z = lookah_join[2] - jps_goal.z;
    int num_steps = int(sqrt(diff_x*diff_x+diff_y*diff_y+diff_z*diff_z)/0.2);
    geometry_msgs::Point interm_pt; //TODO instead use vector and direction
    for (int i = 1; i < num_steps; i++)
    {
      interm_pt.x = diff_x*i/num_steps + jps_goal.x;
      interm_pt.y = diff_y*i/num_steps + jps_goal.y;
      interm_pt.z = diff_z*i/num_steps + jps_goal.z;
      local_traj.markers[0].points.push_back(interm_pt);
    }

    float dt = 0.3;
    ROS_WARN("Get removed seg traj %d %d", local_traj.markers.size(), orig_global_traj_->numSegments());
    orig_global_traj_->getRemovedSegmentTraj(local_traj, traj_lt, traj_seg_num, lookah_join_lt, lookah_join_seg_num, "gt", Eigen::Vector3d(1,0,1), dt);

    //global_traj_marker_pub_.publish(local_traj);

    Eigen::Vector4d limits(max_velocity_, 0, 0, 0);
    ewok::Polynomial3DOptimization<10> to(limits*0.6);
    ewok::Polynomial3DOptimization<10>::Vector3Array path;
    for (int i=1; i<local_traj.markers.size();i++)
    {
      for (int j=0; j<local_traj.markers[i].points.size();j++)
      {
        geometry_msgs::Point pt = local_traj.markers[i].points[j];
        path.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
      }
    }

    local_traj_ = to.computeTrajectory(path);

    /*
    visualization_msgs::MarkerArray local_marker;
    local_traj_->getVisualizationMarkerArray(local_marker, "gt", Eigen::Vector3d(1,0,1));
    global_traj_marker_pub_.publish(local_marker);
    */

    updateTrackingPath(limits, jps_start, local_traj_);
    return true;
  }
  else
  {
    ROS_ERROR("Failed to jps call service"); //TODO handle service not present, planning errors case properly
    return false;
  }
}

void KrTraj::sendCommand()
{
  if(!path_initialized_)
  {
    ROS_DEBUG("Global traj not initialized");
    return;
  }

  ROS_WARN("edrb last update %g sec ago", (ros::Time::now() - last_edrb_move_volume_).toSec());

  if((ros::Time::now() - last_edrb_move_volume_).toSec() > 2.0)
  {
    ROS_WARN("edrb not updated for 2 sec, updating");
    tf::StampedTransform transform;
    try
    {
      listener_->lookupTransform(map_frame_, base_frame_, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
      ROS_INFO("Couldn't get transform");
      ROS_WARN("%s",ex.what());
      return;
    }

    Eigen::Affine3d dT_w_bl;
    tf::transformTFToEigen(transform, dT_w_bl);
    Eigen::Affine3f T_w_bl = dT_w_bl.cast<float>();
    Eigen::Vector3f origin = (T_w_bl * Eigen::Vector4f(0,0,0,1)).head<3>();
    edrbMoveVolume(origin);
  }

  auto t1 = std::chrono::high_resolution_clock::now();
  edrb_->updateDistance();

  auto t2 = std::chrono::high_resolution_clock::now();
  double minf = spline_optimization_->optimize();
  auto t3 = std::chrono::high_resolution_clock::now();

  opt_time << std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count() << " "
      << std::chrono::duration_cast<std::chrono::nanoseconds>(t3-t2).count() << std::endl;

  Eigen::Vector3d pc = spline_optimization_->getFirstOptimizationPoint();

  geometry_msgs::Point pp;
  pp.x = pc[0];
  pp.y = pc[1];
  pp.z = pc[2];
  //ROS_ERROR("Pos cmd %g %g %g", pp.x, pp.y, pp.z);

  //Get closest traj time
  double traj_time = spline_optimization_->getClosestTrajectoryTime(pc, local_time_);
  double min_tim = spline_optimization_->getMinValidTime();
  double max_tim = spline_optimization_->getMaxValidTime();
  double cumulative_traj_time = global_traj_->duration();

  double orig_cumulative_traj_time = orig_global_traj_->duration();

  ROS_INFO("local_tim %g traj_reset_time %g traj_time %g min_tim %g ", local_time_, traj_reset_time_, traj_time, min_tim);
  ROS_WARN("cumulative_traj_time %g max_time %g", cumulative_traj_time, max_tim);
  ROS_INFO("Orig %g", orig_cumulative_traj_time);

  if(max_tim > cumulative_traj_time && (local_time_ > cumulative_traj_time - 1.5))
  {
    ROS_WARN("Reached goal");
    path_tracking_ = false;
    path_initialized_ = false;

    if (tracker_server_->isActive())
    {
      kr_replanning_msgs::TrackPathResult result;
      result.result = true;
      tracker_server_->setSucceeded(result);
    }
    return;
  }

  geometry_msgs::PoseArray optimized_points;
  double dt = dt_;
  //Get the current optimized path
  spline_optimization_->getOptimizedPoints(optimized_points, dt);

  geometry_msgs::Pose first_point, second_point;
  if(optimized_points.poses.size()> 2)
  {
    first_point = optimized_points.poses[0];
    second_point = *(optimized_points.poses.end() - 2);
  }
  else
    ROS_ERROR("vel yaw is invalid, please fix bug");

  tf::Quaternion q;
  tf::quaternionMsgToTF(first_point.orientation, q);
  double vel_yaw = tf::getYaw(q);
  double vel_yaw_traj = vel_yaw;
  double cmd_yaw = last_yaw_;

  if(traj_reset_time_ < 4.0)
    vel_yaw = initial_traj_yaw_;

  //Get current odom transform
  tf::StampedTransform transform;
  try
  {
    listener_->lookupTransform(map_frame_, base_frame_, ros::Time(0), transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_INFO("Couldn't get current odom transform");
    ROS_WARN("%s",ex.what());
    return;
  }

  Eigen::Affine3d dT_o_w;
  tf::transformTFToEigen(transform, dT_o_w);
  Eigen::Affine3f Tf_bl_to_world = dT_o_w.cast<float>();

  //Get current robot yaw
  tf::Quaternion robot_q = transform.getRotation();
  double robot_yaw = tf::getYaw(robot_q);
  tf::Vector3 robot_position =  transform.getOrigin();

  double shortest_ang_robot = angles::shortest_angular_distance(robot_yaw, vel_yaw);
  ROS_WARN("vel_yaw %g %g robot_yaw %g shortest_ang %g", angles::to_degrees(vel_yaw_traj), angles::to_degrees(vel_yaw), angles::to_degrees(robot_yaw), angles::to_degrees(shortest_ang_robot));

  double lookah_time = local_time_ + lookahead_time_;
  Eigen::Vector3d lookah = orig_global_traj_->evaluate(lookah_time);
  Eigen::Vector3d lookah_vel = orig_global_traj_->evaluate(lookah_time, 1);
  Eigen::Vector3d normal_to_traj(-lookah_vel[1],lookah_vel[0], 0 ); //get a vector normal to traj tangent
  normal_to_traj.normalize();

  if(!disable_jps_)
  {
    //Check if traj till lookahead is occupied
    bool trigger_jps = false;
    int num_steps = int(lookahead_time_/0.3); //Check at 0.3 sec resolution
    for (int i = 1; i < num_steps; i++)
    {
      double lt = traj_time + 0.3*i;
      Eigen::Vector3d la = global_traj_->evaluate(lt); //Check currently used global traj
      int ret_occ = edrb_->isOccupied(la.cast<float>());
      if(ret_occ < 0)
        ROS_ERROR("point outside edrb");
      else
      {
        if(ret_occ)
          trigger_jps = true;
      }

      if(trigger_jps)
      {
        ROS_ERROR("Current traj goes through obstacle, call JPS");
        break;
      }
    }

    geometry_msgs::Point lookahead;
    lookahead.x = lookah[0];
    lookahead.y = lookah[1];
    lookahead.z = lookah[2];
    geometry_msgs::PoseStamped min_cost_pt;
    bool found_min_cost = getLookAheadCosts(lookahead, Tf_bl_to_world, normal_to_traj, min_cost_pt);

    if(!found_min_cost)
    {
      ROS_ERROR("******* Case not handled, all lookahead points occupied");
      return;
    }

    //Check if spline optimization is crazy
    bool force_jps = false;
    for(int i = 1; i < optimized_points.poses.size()-1; i++) //Skip the last one
    {
      geometry_msgs::Pose previous_pose = optimized_points.poses[i-1];
      geometry_msgs::Pose current_pose = optimized_points.poses[i];
      double sq_dist = std::pow((current_pose.position.x - previous_pose.position.x),2)
                    + std::pow((current_pose.position.y - previous_pose.position.y),2)
                    + std::pow((current_pose.position.z - previous_pose.position.z),2);

      //ROS_WARN("dist %g %g", sq_dist, std::sqrt(sq_dist));
      float min_separation = 0.5;
      if(sq_dist > min_separation*min_separation)
      {
        ROS_WARN("Something fishy, call JPS? %g last call %g ago", minf, last_time_force_jps_.toSec());
        if((ros::Time::now() - last_time_force_jps_).toSec() > 3.0) //Dont replan faster than 3sec
        {
          ROS_ERROR("Force JPS");
          force_jps = true;
          last_time_force_jps_ = ros::Time::now();
          break;
        }
      }
    }

    if(jps_always_)
    {
      if(traj_reset_time_ > 5.0)
        bool ret = getJpsTraj(local_time_, Tf_bl_to_world, min_cost_pt);
    }
    else
    {
      //if((traj_reset_time_ > 4.0) && ((std::fabs(shortest_ang_robot) > angles::from_degrees(50)) || (minf > 200)))
      if(force_jps || ((traj_reset_time_ > 5.0)  &&
        (trigger_jps || (std::fabs(shortest_ang_robot) > angles::from_degrees(50)) || (minf > 200))))
      {
        ROS_WARN("Calling JPS3D");
        bool ret = getJpsTraj(local_time_, Tf_bl_to_world, min_cost_pt);
        return; //TODO check this
      }
    }

  }

  //If current yaw and cmd_yaw diff is large, just turn in place
  if(std::fabs(shortest_ang_robot) > min_yaw_ang_diff_)
  {
    turn_in_place_ = true;
    ROS_WARN("turning in place to align %g to %g", robot_yaw, vel_yaw);
  }
  else
    turn_in_place_ = false;

  if(minf > 200)
    ROS_WARN("minf %g",minf);

  if(use_goto_)
  {
    boost::array<float ,4> goto_goal = {pp.x, pp.y, pp.z, vel_yaw};
    kr_mav_manager::Vec4 cmd_pos_vec;
    cmd_pos_vec.request.goal = goto_goal;
    if (service_client_.call(cmd_pos_vec))
    {
      //ROS_INFO("Messsage %s", cmd_pos_vec.response.message.c_str());
    }
    else
      ROS_ERROR("Failed to call service Messsage %s", cmd_pos_vec.response.message.c_str());
  }
  else
  {
    //geometry_msgs::PoseArray optimized_points;
    //double dt = dt_;
    //Get the current optimized path
    //spline_optimization_->getOptimizedPoints(optimized_points, dt);

    kr_tracker_msgs::TrajectoryTrackerGoal goal;

    double waypoint_times = 0;
    if(optimized_points.poses.size()>0)
      goal.waypoints.push_back(optimized_points.poses[0]);
    for(int i = 1; i < optimized_points.poses.size(); i++)
    {
      geometry_msgs::Pose previous_pose = optimized_points.poses[i-1];
      geometry_msgs::Pose current_pose = optimized_points.poses[i];
      double sq_dist = std::pow((current_pose.position.x - previous_pose.position.x),2)
                    + std::pow((current_pose.position.y - previous_pose.position.y),2)
                    + std::pow((current_pose.position.z - previous_pose.position.z),2);

      ROS_WARN("dist %g %g", sq_dist, std::sqrt(sq_dist));
      float min_separation = 0.05;
      if(sq_dist > min_separation*min_separation)
      {
        goal.waypoints.push_back(current_pose);
      }

      //goal.waypoint_times.push_back(waypoint_times);
      waypoint_times += dt;
    }

    //goal.waypoint_times.push_back(waypoint_times);

    ROS_WARN("goal sizes %d %d", goal.waypoints.size(), goal.waypoint_times.size());

    traj_tracker_client_.sendGoal(goal, boost::bind(&KrTraj::tracker_done_callback, this, _1, _2), TrajectoryClientType::SimpleActiveCallback(), TrajectoryClientType::SimpleFeedbackCallback());

    kr_tracker_msgs::Transition transition_cmd;
    transition_cmd.request.tracker = traj_tracker_str;

    //TODO only call if active tracker is not the current one?
    if (srv_tracker_transition_.call(transition_cmd) && transition_cmd.response.success)
    {
      ROS_WARN("Current tracker: %s", traj_tracker_str.c_str());
    }

  }

  visualization_msgs::MarkerArray traj_marker;
  spline_optimization_->getMarkers(traj_marker);

  for(int i = 0; i < traj_marker.markers.size(); i++)
    traj_marker.markers[i].header.frame_id = map_frame_;

  current_traj_pub_.publish(traj_marker);

  //~ visualization_msgs::Marker m_dist;
  //~ edrb_->getMarkerDistance(m_dist, 0.5);
  //~ dist_marker_pub_.publish(m_dist);

  last_cmd_ = pc;
  last_yaw_ = vel_yaw;

  if(!turn_in_place_)
  {
    local_time_ += dt_; //Update spline time only if not turning in place
    traj_reset_time_ += dt_;
    spline_optimization_->addLastControlPoint();
  }
}

void KrTraj::track_path_callback()
{
  // If another goal is already active, cancel that goal
  // and track this one instead.
  if (tracker_server_->isActive())
  {
    ROS_INFO("KrTraj track_path goal aborted.");
    tracker_server_->setAborted();
  }
  // Pointer to the goal recieved.
  const auto msg = tracker_server_->acceptNewGoal();

  // If preempt has been requested, then set this goal to preempted
  // and make no changes to the tracker state.
  if (tracker_server_->isPreemptRequested())
  {
    ROS_INFO("KrTraj track_path goal preempted.");
    tracker_server_->setPreempted();
    return;
  }

  ROS_WARN("KrTraj TrackPathAction goal received");
  //const nav_msgs::Path pth = msg->path;
  nav_msgs::Path::ConstPtr ptr(new nav_msgs::Path(msg->path));
  processPath(ptr);
}

void KrTraj::preempt_callback()
{
  if (tracker_server_->isActive())
  {
    ROS_INFO("KrTraj track_path going to goal aborted.");
    tracker_server_->setAborted();
  }
  else
  {
    ROS_INFO("KrTraj track_path going to goal preempted.");
    tracker_server_->setPreempted();
  }

  path_tracking_ = false;
  path_initialized_ = false;
}

void KrTraj::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
  // and track this one instead.
  if (tracker_server_->isActive())
  {
    ROS_INFO("KrTraj track_path goal aborted.");
    tracker_server_->setAborted();
  }

  processPath(msg);
}

void KrTraj::processPath(const nav_msgs::Path::ConstPtr& msg)
{
  if(msg->header.frame_id != map_frame_)
  {
    ROS_ERROR("Global path frame %s does not match %s frame",msg->header.frame_id.c_str(), map_frame_.c_str());
    return;
  }

  ewok::Polynomial3DOptimization<10>::Vector3Array path;

  if(use_current_start_)
  {
    if(msg->poses.size() < 1)
    {
      ROS_ERROR("Need atleast single waypoints to start tracking");
      if (tracker_server_->isActive())
      {
        ROS_INFO("KrTraj track_path goal aborted.");
        tracker_server_->setAborted();
      }
      return;
    }

    //Get current odom transform
    tf::StampedTransform transform;
    try
    {
      listener_->lookupTransform(map_frame_, base_frame_, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("Couldn't get current odom transform");
      ROS_WARN("%s",ex.what());
      if (tracker_server_->isActive())
      {
        ROS_INFO("KrTraj track_path goal aborted.");
        tracker_server_->setAborted();
      }
      return;
    }

    //Add the current MAV position as first waypoint
    tf::Transform odom_tf(transform.getBasis(),transform.getOrigin());
    Eigen::Vector3d current_odom_wp(odom_tf.getOrigin().getX(), odom_tf.getOrigin().getY(), odom_tf.getOrigin().getZ());

    if(msg->poses.size() == 1)
      path.push_back(current_odom_wp); //Just single wp specified, safe to current robot pose as start
    else
    {
      //Atleast 2 wp specified, safe to access based on index
      geometry_msgs::PoseStamped ps = msg->poses[0];
      Eigen::Vector3d first_wp(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);

      ps = msg->poses[1];
      Eigen::Vector3d second_wp(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);

      Eigen::Vector3d vec1 = second_wp - first_wp;
      Eigen::Vector3d vec2 = first_wp - current_odom_wp;

      vec1.normalize();
      vec2.normalize();
      float ang = std::acos(vec1.dot(vec2));

      ROS_WARN("Angle between odom and initial yaw %g", angles::to_degrees(ang));
      if(std::fabs(ang) > angles::from_degrees(120))
        ROS_WARN("start almost between first two wp, skipping adding");
      else
        path.push_back(current_odom_wp);
    }

  }
  else
  {
    if(msg->poses.size() < 2)
    {
      ROS_ERROR("Need atleast two waypoints to start tracking");
      if (tracker_server_->isActive())
      {
        ROS_INFO("KrTraj track_path goal aborted.");
        tracker_server_->setAborted();
      }
      return;
    }
  }

  for(int i = 0; i < msg->poses.size(); i++)
  {
    geometry_msgs::PoseStamped ps = msg->poses[i];
    path.push_back(Eigen::Vector3d(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z));
  }

  Eigen::Vector4d limits(max_velocity_, max_acceleration_, 0, 0);
  ewok::Polynomial3DOptimization<10> to(limits*0.6);
  global_traj_ = to.computeTrajectory(path);
  orig_global_traj_ = global_traj_;

  visualization_msgs::MarkerArray traj_marker;
  global_traj_->getVisualizationMarkerArray(traj_marker, "gt", Eigen::Vector3d(1,0,1));
  for(int i = 0; i < traj_marker.markers.size(); i++)
    traj_marker.markers[i].header.frame_id = map_frame_;

  global_traj_marker_pub_.publish(traj_marker);

  //Get initial traj yaw
  std::vector<geometry_msgs::Point> traj_pts = traj_marker.markers[0].points;
  geometry_msgs::Point start = traj_pts[0]; //TODO range check?

  initial_traj_yaw_ = 0.0;
  if(traj_pts.size()>2)
  {
    geometry_msgs::Point pt1 = traj_pts[0];
    geometry_msgs::Point pt2 = traj_pts[1];
    initial_traj_yaw_ = std::atan2(pt2.y - pt1.y, pt2.x - pt1.x);
  }

  ROS_WARN("Initial yaw %g", angles::to_degrees(initial_traj_yaw_));

  /*
  geometry_msgs::Point start;// = traj_pts[0]; //TODO range check?
  start.x = odom_tf.getOrigin().getX();
  start.y = odom_tf.getOrigin().getY();
  start.z = odom_tf.getOrigin().getZ();
  */

  ROS_INFO("Received new waypoints, num segments %d", traj_marker.markers.size());

  setTrackingPath(limits, start, global_traj_);
}

void KrTraj::setTrackingPath(const Eigen::Vector4d& limits, const geometry_msgs::Point& start, ewok::PolynomialTrajectory3D<10>::Ptr& traj)
{
  spline_optimization_.reset(new ewok::UniformBSpline3DOptimization<6>(traj, dt_));

  float dt = 0.0;
  for (int i = 0; i < num_opt_points_; i++)
  {
    Eigen::Vector3d lookah = traj->evaluate(dt);
    spline_optimization_->addControlPoint(lookah);
    //spline_optimization_->addControlPoint(Eigen::Vector3d(start.x, start.y, start.z));

    dt += 0.05;
  }
  spline_optimization_->setNumControlPointsOptimized(num_opt_points_);
  spline_optimization_->setDistanceBuffer(edrb_);
  spline_optimization_->setDistanceThreshold(distance_threshold_);
  spline_optimization_->setLimits(limits);

  path_initialized_ = true;
  path_tracking_ = true;
  local_time_ = 0.0;
  traj_reset_time_ = 0.0;
}

void KrTraj::updateTrackingPath(const Eigen::Vector4d& limits, const geometry_msgs::Point& start, ewok::PolynomialTrajectory3D<10>::Ptr& traj)
{
  global_traj_ = traj;

  visualization_msgs::MarkerArray traj_marker;
  traj->getVisualizationMarkerArray(traj_marker, "gt", Eigen::Vector3d(1,0,1));
  for(int i = 0; i < traj_marker.markers.size(); i++)
    traj_marker.markers[i].header.frame_id = map_frame_;
  global_traj_marker_pub_.publish(traj_marker);

  //Get initial traj yaw
  std::vector<geometry_msgs::Point> traj_pts = traj_marker.markers[0].points;

  initial_traj_yaw_ = 0.0;
  if(traj_pts.size()>2)
  {
    geometry_msgs::Point pt1 = traj_pts[0];
    geometry_msgs::Point pt2 = traj_pts[1];
    initial_traj_yaw_ = std::atan2(pt2.y - pt1.y, pt2.x - pt1.x);
  }
  ROS_WARN("Initial yaw %g", angles::to_degrees(initial_traj_yaw_));

  Eigen::Vector3d st(start.x, start.y, start.z);
  spline_optimization_.reset(new ewok::UniformBSpline3DOptimization<6>(st, traj, dt_));
  float dt = 0.0;
  for (int i = 0; i < num_opt_points_; i++)
  {
    Eigen::Vector3d lookah = traj->evaluate(dt);
    spline_optimization_->addControlPoint(lookah);
    //spline_optimization_->addControlPoint(st);
    dt += 0.05;
  }

  spline_optimization_->setNumControlPointsOptimized(num_opt_points_);
  spline_optimization_->setDistanceBuffer(edrb_);
  spline_optimization_->setDistanceThreshold(distance_threshold_);
  spline_optimization_->setLimits(limits);

  path_initialized_ = true;
  path_tracking_ = true;
  traj_reset_time_ = 0.0;

  ROS_INFO("Received new waypoints, num segments");// %d", traj_marker.markers.size());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "krui_traj_ex");
  ros::NodeHandle pnh;

  double dt;
  pnh.param("dt", dt, 0.5);
  ros::Rate loop_rate(1.0/dt);

  KrTraj kr_traj;

  while (ros::ok())
  {
    ros::Time start_tim = ros::Time::now();

    kr_traj.sendCommand();

    ros::Time curr_tim = ros::Time::now();
    ros::Duration diff = curr_tim - start_tim;
    //ROS_ERROR("send_command dur %g", diff.toSec());

    ros::spinOnce();
    loop_rate.sleep();

    curr_tim = ros::Time::now();
    diff = curr_tim - start_tim;
    //ROS_ERROR("total command dur %g", diff.toSec());
  }
}
