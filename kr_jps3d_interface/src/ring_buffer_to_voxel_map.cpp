#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <kr_replanning_msgs/VoxelMap.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

class RingBufferToVoxelMap
{
public:
  RingBufferToVoxelMap();
private:
  void ringBufferCallback(const visualization_msgs::Marker::ConstPtr& msg);
  int calculate_id(geometry_msgs::Point dim, int x_ind, int y_ind, int z_ind);

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber ring_buffer_sub_;
  ros::Publisher voxel_pub_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;

  int voxel_map_size_;
  double voxel_map_resolution_;
  bool inflate_;
  std::string world_frame_,planning_frame_;

};

RingBufferToVoxelMap::RingBufferToVoxelMap()
{
  pnh_ = ros::NodeHandle("~");

  pnh_.param("voxel_map_size", voxel_map_size_, 8);
  pnh_.param("resolution", voxel_map_resolution_, 0.0);
  pnh_.param<std::string>("world_frame", world_frame_, "world");
  pnh_.param<std::string>("planning_frame", planning_frame_, "voxel_map");
  pnh_.param("inflate_map", inflate_, false);

  if (voxel_map_resolution_ == 0.0)
  {
    ROS_ERROR("Failed to get ringbuffer resolution %f", voxel_map_resolution_);
    voxel_map_resolution_ = 0.1;
  }

  voxel_pub_ = nh_.advertise<kr_replanning_msgs::VoxelMap>("rb_to_voxel_map", 1);
  ring_buffer_sub_ = nh_.subscribe<visualization_msgs::Marker>("ring_buffer/occupied", 1, boost::bind(&RingBufferToVoxelMap::ringBufferCallback, this, _1));

}

int RingBufferToVoxelMap::calculate_id(geometry_msgs::Point dim, int x_ind, int y_ind, int z_ind)
{
  int id = x_ind + dim.x * y_ind + dim.x * dim.y * z_ind;
  if (x_ind < dim.x && y_ind < dim.y && z_ind < dim.z && x_ind >= 0 && y_ind >= 0 && z_ind >= 0)
    return id;
  else
  {
    ROS_ERROR("Ring buffer data not in dimensions of map");
    return -1;
  }
}

void RingBufferToVoxelMap::ringBufferCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
  visualization_msgs::Marker marker = *msg;
  kr_replanning_msgs::VoxelMap voxel_map;

  voxel_map.header.frame_id = planning_frame_; //TODO add stamp

  voxel_map.resolution = voxel_map_resolution_;

  double marker_origin_pt[3] = {marker.pose.position.x,marker.pose.position.y, marker.pose.position.z};
  double lower_bottom_corner[3] = {-3.15, -3.25, -3.15}; //make class memeber
  int infl_rad[3] = {1,1,1};

  geometry_msgs::Point dim;
  dim.x = int(voxel_map_size_/voxel_map.resolution);
  dim.y = int(voxel_map_size_/voxel_map.resolution);
  dim.z = int(voxel_map_size_/voxel_map.resolution);
  //~ ROS_INFO("Dimension %f,%f,%f",dim.x,dim.y,dim.z);
  voxel_map.dim = dim;
  voxel_map.data.resize( dim.x*dim.y*dim.z);

  geometry_msgs::Point pt;
  for(std::vector<geometry_msgs::Point>::iterator it = marker.points.begin(); it != marker.points.end(); ++it)
  {
    pt = *it;
    //~ int x_ind = std::round((pt.x + marker_origin_pt[0] + voxel_map_size_/2)/voxel_map.resolution -.5);
    //~ int y_ind = std::round((pt.y + marker_origin_pt[1]+ voxel_map_size_/2)/voxel_map.resolution -.5);
    //~ int z_ind = std::round((pt.z + marker_origin_pt[2]+ voxel_map_size_/2)/voxel_map.resolution -.5);
    int x_ind = std::round((pt.x + lower_bottom_corner[0] + voxel_map_size_/2)/voxel_map.resolution -.5);
    int y_ind = std::round((pt.y + lower_bottom_corner[1] + voxel_map_size_/2)/voxel_map.resolution -.5);
    int z_ind = std::round((pt.z + lower_bottom_corner[2] + voxel_map_size_/2)/voxel_map.resolution -.5);
    int id  = calculate_id(dim,x_ind,y_ind,z_ind);
    if (id >= 0){
      voxel_map.data[id] = 100;

      if(inflate_)
      {
        for (int l=-infl_rad[0]; l<=infl_rad[0];l++){
          for (int m=-infl_rad[1]; m<=infl_rad[1];m++){
            for (int n=-infl_rad[2]; n<=infl_rad[2];n++){
                  id = calculate_id (dim, x_ind +l, y_ind +m, z_ind +n);
                  if (id>=0) voxel_map.data[id]=100;
            }
          }
        }
      }
    }
    else
    {
      ROS_INFO("World point %f %f %f",(pt.x + marker_origin_pt[0]),(pt.y + marker_origin_pt[1]),(pt.z + marker_origin_pt[2]));
      ROS_INFO("Index point %f %f %f",(pt.x + marker_origin_pt[0])/voxel_map.resolution,(pt.y + marker_origin_pt[1])/voxel_map.resolution,(pt.z + marker_origin_pt[2])/voxel_map.resolution);
      ROS_INFO("Int Index point %d %d %d",x_ind,y_ind,z_ind);
    }
  }

  geometry_msgs::TransformStamped static_transformStamped;
  static_transformStamped.header.stamp = marker.header.stamp;
  static_transformStamped.header.frame_id = world_frame_;
  static_transformStamped.child_frame_id = planning_frame_;
  static_transformStamped.transform.translation.x = -voxel_map_size_/2 + marker_origin_pt[0] - lower_bottom_corner[0];
  static_transformStamped.transform.translation.y = -voxel_map_size_/2 + marker_origin_pt[1] - lower_bottom_corner[1];
  static_transformStamped.transform.translation.z = -voxel_map_size_/2 + marker_origin_pt[2] - lower_bottom_corner[2];
  static_transformStamped.transform.rotation.w = 1;
  static_broadcaster_.sendTransform(static_transformStamped);

  //~ voxel_map.origin.x = marker_origin_pt[0] +3.2;
  //~ voxel_map.origin.y = marker_origin_pt[1] +3.2;
  //~ voxel_map.origin.z = marker_origin_pt[2] +3.2;
  voxel_pub_.publish(voxel_map);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ring_buffer_to_voxel_map");

  RingBufferToVoxelMap rbvm;
  ros::spin();
  return 0;
}
