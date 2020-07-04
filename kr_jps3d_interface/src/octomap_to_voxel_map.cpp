#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <kr_replanning_msgs/VoxelMap.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class OctomapToVoxelMap
{
public:
  OctomapToVoxelMap();
private:
  void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg);
  int calculate_id(geometry_msgs::Point dim, int x_ind, int y_ind, int z_ind);

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber octomap_sub_;
  ros::Publisher voxel_pub_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  std::string cloud_frame_,planning_frame_;

};

OctomapToVoxelMap::OctomapToVoxelMap()
{
  pnh_ = ros::NodeHandle("~");
  pnh_.param<std::string>("cloud_frame", cloud_frame_, "cloud_frame");
  pnh_.param<std::string>("planning_frame", planning_frame_, "voxel_map");

  voxel_pub_ = nh_.advertise<kr_replanning_msgs::VoxelMap>("voxel_map", 1);
  octomap_sub_ = nh_.subscribe<octomap_msgs::Octomap>("octomap_full", 1, boost::bind(&OctomapToVoxelMap::octomapCallback, this, _1));
}


int OctomapToVoxelMap::calculate_id(geometry_msgs::Point dim, int x_ind, int y_ind, int z_ind)
{
  int id = x_ind + dim.x * y_ind + dim.x * dim.y * z_ind;
  if (x_ind < dim.x && y_ind < dim.y && z_ind < dim.z && x_ind >= 0 && y_ind >= 0 && z_ind >= 0)
    return id;
  else
  {
    //~ ROS_ERROR("Octomap data not in dimensions of map");
    return -1;
  }
}

void OctomapToVoxelMap::octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
  octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
  double voxel_map_size_x,voxel_map_size_y,voxel_map_size_z;
	double min_x,min_y,min_z;
	double max_x,max_y,max_z;
  geometry_msgs::Point dim;

	octree->getMetricMin(min_x,min_y,min_z);
	octree->getMetricSize(voxel_map_size_x,voxel_map_size_y,voxel_map_size_z);

  kr_replanning_msgs::VoxelMap voxel_map;
	voxel_map.resolution = octree->getResolution();
  dim.x = int(voxel_map_size_x/voxel_map.resolution)+1;
  dim.y = int(voxel_map_size_y/voxel_map.resolution)+1;
  dim.z = int(voxel_map_size_z/voxel_map.resolution)+1;
  voxel_map.dim = dim;
	voxel_map.data.resize(dim.x * dim.y * dim.z, 0); // initialize as free map, free cell has 0 occupancy
  int infl_rad[3] = {1,1,1};
  int id;

	for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end=octree->end_leafs(); it!= end; ++it){
    if (!octree->isNodeOccupied(*it)) continue;
		int x_ind = int((it.getX()-min_x)/voxel_map.resolution);
		int y_ind = int((it.getY()-min_y)/voxel_map.resolution);
		int z_ind = int((it.getZ()-min_z)/voxel_map.resolution);
		double size  = it.getSize();
		id = calculate_id(dim,x_ind, y_ind , z_ind);
    int size_ind = (int)(size/voxel_map.resolution);

    if (size_ind<=1) {
        if (id>=0) voxel_map.data[id] =100;
        for (int l=-infl_rad[0]; l<=infl_rad[0];l++){
          for (int m=-infl_rad[1]; m<=infl_rad[1];m++){
            for (int n=-infl_rad[2]; n<=infl_rad[2];n++){
              id = calculate_id (dim, x_ind +l, y_ind +m, z_ind +n);
              if (id>=0) voxel_map.data[id]=100;
            }
          }
        }

    }
    else {
      for (int i=0; i<=size_ind; i++){
        for (int j=0; j<=size_ind; j++){
          for (int k=0; k<=size_ind; k++){
              id = calculate_id (dim, x_ind - size/voxel_map.resolution/2 + i, y_ind - size/voxel_map.resolution/2 + j, z_ind - size/voxel_map.resolution/2 + k);
              if (id>=0) voxel_map.data[id] =100;
              for (int l=-infl_rad[0]; l<=infl_rad[0];l++){
                for (int m=-infl_rad[1]; m<=infl_rad[1];m++){
                  for (int n=-infl_rad[2]; n<=infl_rad[2];n++){
                    id = calculate_id (dim, x_ind - size/voxel_map.resolution/2 + i +l, y_ind - size/voxel_map.resolution/2 + j+m, z_ind - size/voxel_map.resolution/2 + k+n);
                    if (id>=0) voxel_map.data[id]=100;
                  }
                }
              }
          }
        }
      }
    }
  }
  // add extra planes to voxel_map
  for (int i=0; i<dim.x; i++){
    for (int j=0; j<dim.y; j++){
      //bottom plane
      id = calculate_id(dim,i,j,7);
      if (id>=0) voxel_map.data[id] =100;
      //top of model
      if (j>3.8/voxel_map.resolution){
        id = calculate_id (dim,i,j,int(2.6/voxel_map.resolution));
        if (id>=0) voxel_map.data[id] =100;
      }
    }
    //xz plane wall to force the correct direction around the model
    for (int k=0; k<dim.z; k++){
      if (i<10.3/voxel_map.resolution){
        id = calculate_id(dim,i,int(5.5/voxel_map.resolution),k);
        if (id>=0) voxel_map.data[id] =100;
      }
    }

  }

  voxel_map.header.frame_id = planning_frame_ ;
  voxel_pub_.publish(voxel_map);

  geometry_msgs::TransformStamped static_transformStamped;
  static_transformStamped.header.stamp = msg->header.stamp;
  static_transformStamped.header.frame_id = cloud_frame_;
  static_transformStamped.child_frame_id = planning_frame_;
  static_transformStamped.transform.translation.x = min_x;
  static_transformStamped.transform.translation.y = min_y;
  static_transformStamped.transform.translation.z = min_z;
  static_transformStamped.transform.rotation.w = 1;
  static_broadcaster_.sendTransform(static_transformStamped);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "octomap_to_voxel_map");

  OctomapToVoxelMap ctvm;
  ros::spin();
  return 0;
}
