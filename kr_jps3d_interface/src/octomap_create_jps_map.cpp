#include <cmath>
#include <ros/ros.h>
//~ #include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
//~ #include <octomap_server/OctomapServer.h>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

ros::Publisher marker_pub;
int calculate_id(std::vector<int> dim, int x_ind, int y_ind, int z_ind){
	int id = x_ind + dim[0] * y_ind + dim[0] * dim[1] * z_ind;
	if (x_ind<dim[0] && y_ind<dim[1] && z_ind<dim[2] && x_ind>=0 && y_ind>=0 && z_ind>=0)
		return id;
	else return -1;
}
void create_map_2d(std::vector<int> data, double res, std::vector<int> dim){

  std::vector<double> start{0.5, 0.5};
  std::vector<double> goal{1.5, 0.5};
  // Create a map
  std::vector<double> origin{0, 0};
  std::vector<int> data_2d(data.begin() + dim[0] * dim[1] * 5, data.begin() + dim[0] * dim[1] * 6);

  YAML::Emitter out;
  out << YAML::BeginSeq;
  // Encode start coordinate
  out << YAML::BeginMap;
  out << YAML::Key << "start" << YAML::Value << YAML::Flow << start;
  out << YAML::EndMap;
  // Encode goal coordinate
  out << YAML::BeginMap;
  out << YAML::Key << "goal" << YAML::Value << YAML::Flow << goal;
  out << YAML::EndMap;
  // Encode origin coordinate
  out << YAML::BeginMap;
  out << YAML::Key << "origin" << YAML::Value << YAML::Flow << origin;
  out << YAML::EndMap;
  // Encode dimension as number of cells
  out << YAML::BeginMap;
  out << YAML::Key << "dim" << YAML::Value << YAML::Flow << dim;
  out << YAML::EndMap;
  // Encode resolution
  out << YAML::BeginMap;
  out << YAML::Key << "resolution" << YAML::Value << res;
  out << YAML::EndMap;
  // Encode occupancy
  out << YAML::BeginMap;
  out << YAML::Key << "data" << YAML::Value << YAML::Flow << data_2d;
  out << YAML::EndMap;

  out << YAML::EndSeq;

  std::ofstream file;
  std::string filename("octomap_2d.yaml");
  file.open(filename);
  file << out.c_str();
  file.close();
  std::cout << "wrote yaml file " << filename << std::endl;

}
void create_map(octomap::OcTree* octree){
	double res = octree->getResolution();
	double x,y,z;
	double min_x,min_y,min_z;
	double max_x,max_y,max_z;
	octree->getMetricMin(min_x,min_y,min_z);
  ROS_INFO("%f %f %f",min_x,min_y,min_z);

	octree->getMetricSize(x,y,z);
	std::vector<int> dim{int(x/res),int(y/res),int(z/res)};
	std::vector<int> data; // occupancy data, the subscript follows: id = x + dim.x * y + dim.x * dim.y * z; dim.x +dim.x*dim.y + dim.x*dim.y*dim.z
	data.resize(dim[0] * dim[1] * dim[2], 0); // initialize as free map, free cell has 0 occupancy
	int infl_rad[3] = {1,1,1};
	for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end=octree->end_leafs(); it!= end; ++it){
    if (!octree->isNodeOccupied(*it)) continue;
		int x_ind = (it.getX()-min_x)/res;
		int y_ind = (it.getY()-min_y)/res;
		int z_ind = (it.getZ()-min_z)/res;
		double size  = it.getSize();
		int id = calculate_id(dim,x_ind, y_ind , z_ind);
		if (size/res==1){
			if (id>=0) data[id] =100;
			for (int l=-infl_rad[0]; l<=infl_rad[0];l++){
				for (int m=-infl_rad[1]; m<=infl_rad[1];m++){
					for (int n=-infl_rad[2]; n<=infl_rad[2];n++){
						id = calculate_id (dim, x_ind +l, y_ind +m, z_ind +n);
						if (id>=0) data[id]=100;
					}
				}
			}
		}
		else{
        for (int i=0; i<=(size/res); i++){
			for (int j=0; j<=(size/res); j++){
				for (int k=0; k<=(size/res); k++){
					id = calculate_id (dim, x_ind - size/res/2 + i, y_ind - size/res/2 + j, z_ind - size/res/2 + k);
					if (id>=0) data[id] =100;
					for (int l=-infl_rad[0]; l<=infl_rad[0];l++){
						for (int m=-infl_rad[1]; m<=infl_rad[1];m++){
							for (int n=-infl_rad[2]; n<=infl_rad[2];n++){
								id = calculate_id (dim, x_ind - size/res/2 + i +l, y_ind - size/res/2 + j+m, z_ind - size/res/2 + k+n);
								if (id>=0) data[id]=100;
							}
						}
					}
				}
			}
		}
		}
	}
  std::vector<double> start{0.5, 0.5, 0.5};
  std::vector<double> goal{1.5, 0.5, 0.5};
  // Create a map
  std::vector<double> origin{0, 0, 0};

  YAML::Emitter out;
  out << YAML::BeginSeq;
  // Encode start coordinate
  out << YAML::BeginMap;
  out << YAML::Key << "start" << YAML::Value << YAML::Flow << start;
  out << YAML::EndMap;
  // Encode goal coordinate
  out << YAML::BeginMap;
  out << YAML::Key << "goal" << YAML::Value << YAML::Flow << goal;
  out << YAML::EndMap;
  // Encode origin coordinate
  out << YAML::BeginMap;
  out << YAML::Key << "origin" << YAML::Value << YAML::Flow << origin;
  out << YAML::EndMap;
  // Encode dimension as number of cells
  out << YAML::BeginMap;
  out << YAML::Key << "dim" << YAML::Value << YAML::Flow << dim;
  out << YAML::EndMap;
  // Encode resolution
  out << YAML::BeginMap;
  out << YAML::Key << "resolution" << YAML::Value << res;
  out << YAML::EndMap;
  // Encode occupancy
  out << YAML::BeginMap;
  out << YAML::Key << "data" << YAML::Value << YAML::Flow << data;
  out << YAML::EndMap;

  out << YAML::EndSeq;

  std::ofstream file;
  std::string filename("octomap.yaml");
  file.open(filename);
  file << out.c_str();
  file.close();
  std::cout << "wrote yaml file " << filename << std::endl;
  create_map_2d(data,res,dim);
}

void publish_markers(octomap::OcTree* octree){

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    double res = octree->getResolution();
    marker.scale.x = res;
    marker.scale.y = res;
    marker.scale.z = res;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end=octree->end_leafs(); it!= end; ++it){
        geometry_msgs::Point point;
		point.x = it.getX();
		point.y = it.getY();
		point.z = it.getZ();
		double size  = it.getSize();
		if (size/res==1) marker.points.push_back(point);
		else{
		float x = point.x;
		float y = point.y;
		float z = point.z;
        for (int i=1; i<=(size/res); i++){
			for (int j=1; j<=(size/res); j++){
				for (int k=1; k<=(size/res); k++){
					point.x = x + (0.5 + ((i-1)/2))*pow(-1,i)*res;
					point.y = y + (0.5 + ((j-1)/2))*pow(-1,j)*res;
					point.z = z + (0.5 + ((k-1)/2))*pow(-1,k)*res;
					marker.points.push_back(point);
					}
				}
			}
		}
    }
    marker_pub.publish( marker );
    //~ while (ros::ok){
        //~ marker_pub.publish( marker );
        //~ ros::Duration(5).sleep();
    //~ }

}
void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg){

    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
    publish_markers(octree);
    create_map(octree);

    //~ ros::shutdown();


}

int main(int argc, char** argv) {
    ros::init(argc, argv, "octomap_create_jps_map");
    ros::NodeHandle nh("~");
    ros::Subscriber sub = nh.subscribe("/octomap_full", 1000, octomapCallback);
    //~ OcTree* octree = new OcTree("~/kr_jps3d_interface/temp_octomap.bt");
    marker_pub = nh.advertise<visualization_msgs::Marker>("octomap_conversion", 1);

    ros::spin();
    return 0;
}
