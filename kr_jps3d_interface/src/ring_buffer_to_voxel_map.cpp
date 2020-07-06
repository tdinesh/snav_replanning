#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <kr_replanning_msgs/VoxelMap.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
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
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  int voxel_map_size_;
  double voxel_map_resolution_;
  bool inflate_;
  bool floor_ceil_;
  double ceiling_height_, floor_height_;
  std::string world_frame_,planning_frame_, base_frame_;
};

RingBufferToVoxelMap::RingBufferToVoxelMap()
{
  pnh_ = ros::NodeHandle("~");

  pnh_.param("voxel_map_size", voxel_map_size_, 8);
  pnh_.param("resolution", voxel_map_resolution_, 0.1);
  pnh_.param("inflate_map", inflate_, false);
  pnh_.param<std::string>("world_frame", world_frame_, "world");
  pnh_.param<std::string>("planning_frame", planning_frame_, "voxel_map");
  pnh_.param<std::string>("base_frame", base_frame_, "voxel_map");

  pnh_.param("floor_ceiling", floor_ceil_, false);
  pnh_.param("ceiling_height", ceiling_height_, 3.0);
  pnh_.param("floor_height", floor_height_, 0.1);

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
    //ROS_ERROR("Ring buffer data not in dimensions of map");
    return -1;
  }
}

void RingBufferToVoxelMap::ringBufferCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
  visualization_msgs::Marker marker = *msg; //marker points are in world_frame

  kr_replanning_msgs::VoxelMap voxel_map;
  voxel_map.header.frame_id = planning_frame_;
  voxel_map.header.stamp = marker.header.stamp;
  voxel_map.resolution = voxel_map_resolution_;

  int infl_rad[3] = {1,1,1}; //make class member

  //voxel map dimensions
  geometry_msgs::Point dim;
  dim.x = int(voxel_map_size_/voxel_map.resolution);
  dim.y = int(voxel_map_size_/voxel_map.resolution);
  dim.z = int(voxel_map_size_/voxel_map.resolution);
  voxel_map.dim = dim;
  voxel_map.data.resize(dim.x * dim.y * dim.z);
  // ROS_INFO("Dimension %f,%f,%f",dim.x,dim.y,dim.z);

  /*
  voxel_map.origin.x = marker.pose.position.x;
  voxel_map.origin.y = marker.pose.position.y;
  voxel_map.origin.z = marker.pose.position.z;*/

  //ROS_WARN("Voxel origin %f %f %f",voxel_map.origin.x, voxel_map.origin.y, voxel_map.origin.z);
  //ROS_WARN("Marke origin %f %f %f", marker.pose.position.x,marker.pose.position.y, marker.pose.position.z);

  geometry_msgs::Point pt;
  for(std::vector<geometry_msgs::Point>::iterator it = marker.points.begin(); it != marker.points.end(); ++it)
  {
    //ring buffer points are in marker origin frame
    pt = *it;

    /*
    //Occupied world point
    geometry_msgs::Point world_pt;
    world_pt.x = pt.x + marker.pose.position.x;
    world_pt.y = pt.y + marker.pose.position.y;
    world_pt.z = pt.z + marker.pose.position.z;
    */

    int x_ind = std::round((pt.x)/voxel_map.resolution - 0.5);
    int y_ind = std::round((pt.y)/voxel_map.resolution - 0.5);
    int z_ind = std::round((pt.z)/voxel_map.resolution - 0.5);

    int id  = calculate_id(dim, x_ind, y_ind, z_ind);
    if(id >= 0)
    {
      voxel_map.data[id] = 100;

      if(inflate_)
      {
        for (int l=-infl_rad[0]; l<=infl_rad[0]; l++)
        {
          for (int m=-infl_rad[1]; m<=infl_rad[1]; m++)
          {
            for (int n=-infl_rad[2]; n<=infl_rad[2]; n++)
            {
              id = calculate_id(dim, x_ind + l, y_ind + m, z_ind + n);
              if (id>=0)
                voxel_map.data[id]=100;
            }
          }
        }
      }
    }
    else
    {
      //ROS_WARN("World point %f %f %f",world_pt.x, world_pt.y, world_pt.z);
      //ROS_WARN("Index point %f %f %f",(world_pt.x - voxel_map.origin.x)/voxel_map.resolution, (world_pt.y - voxel_map.origin.y)/voxel_map.resolution, (world_pt.z - voxel_map.origin.z)/voxel_map.resolution);
      //ROS_WARN("Int Index point %d %d %d",x_ind, y_ind, z_ind);
    }
  }

  if(floor_ceil_)
  {
    //Points in planner frame
    double p_pt_floor = floor_height_ - marker.pose.position.z;
    double p_pt_ceiling = ceiling_height_ - marker.pose.position.z;

    int z_ind_floor = std::round((p_pt_floor)/voxel_map.resolution - 0.5);
    int z_ind_ceiling = std::round((p_pt_ceiling)/voxel_map.resolution - 0.5);

    for(int x = 0; x < dim.x; x ++)
    {
      for(int y = 0; y < dim.y; y ++)
      {
        int id = -1;

        id = calculate_id(dim, x, y, z_ind_floor); //calculate id from map indices
        if (id>=0)
          voxel_map.data[id]=100; //id=-1 if out of range of map

        id = calculate_id(dim, x, y, z_ind_ceiling); //calculate id from map indices
        if (id>=0)
          voxel_map.data[id]=100; //id=-1 if out of range of map
      }
    }
  }

  //Planner frame is lower corner from base_link aligned with world
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.stamp = marker.header.stamp;
  transform_stamped.header.frame_id = world_frame_;
  transform_stamped.child_frame_id = planning_frame_;
  transform_stamped.transform.translation.x = marker.pose.position.x;
  transform_stamped.transform.translation.y = marker.pose.position.y;
  transform_stamped.transform.translation.z = marker.pose.position.z;
  transform_stamped.transform.rotation.w = 1.0;
  tf_broadcaster_.sendTransform(transform_stamped);

  //voxel_map.origin.x = 0.0; voxel_map.origin.y = 0.0;voxel_map.origin.z = 0.0;
  voxel_pub_.publish(voxel_map);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ring_buffer_to_voxel_map");

  RingBufferToVoxelMap rbvm;
  ros::spin();
  return 0;
}
