#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <map>
#include <string>
#include <iostream>
#include <fstream>


# define PI 3.14159265358979323846


#define TIBIA_MIN -0.58
#define TIBIA_MAX 1.55
#define FEMUR_MIN -0.75
#define FEMUR_MAX 1.55
#define COXA_MIN -1.55
#define COXA_MAX 0.66

std::map<std::string, double> robot_state;

void initializeStatePublisher(){
  std::string coxa ("coxa_joint_");
  std::string femur ("femur_joint_");
  std::string tibia ("tibia_joint_");
    
  for(int i=1; i <=6; i++){
    std::ostringstream snum;
    snum << i;
    std::string num = snum.str();
    robot_state.insert(std::pair<std::string, double>(coxa+num, 0.0));
    robot_state.insert(std::pair<std::string, double>(femur+num, 0.0));
    robot_state.insert(std::pair<std::string, double>(tibia+num, 0.0));
  }
}

int main(int argc, char ** argv){
  initializeStatePublisher();
  
  
  ros::init(argc, argv, "state_publisher");
  ROS_INFO("Robot State Publisher Initialized");
  ros::NodeHandle node;
  ros::Publisher joint_pub =  node.advertise<sensor_msgs::JointState>("joint_states",1);

  ros::Rate loop_rate(100);
  sensor_msgs::JointState joint_state;

  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(18);
  joint_state.position.resize(18);
  int i=0;
  for (std::map<std::string, double>::iterator it=robot_state.begin(); it!=robot_state.end(); ++it, i++){
    joint_state.name[i] = it->first.c_str();
    joint_state.position[i] = it->second;
  }

  joint_pub.publish(joint_state);
  loop_rate.sleep();
 

  tf::TransformListener listener ;
  tf::StampedTransform transform ;
  ros::Duration(2).sleep(); // sleep for half a second
 

  std::ofstream dataFile;
  dataFile.open ("data.txt");
  dataFile <<"#coxa femur tibia x y z\n";

  double increment =  1 * PI/180; //12:16 

  double coxa_value = COXA_MIN;
  while(coxa_value < COXA_MAX){
    double femur_value = FEMUR_MIN;
    
    while(femur_value < FEMUR_MAX){
      double tibia_value = TIBIA_MIN;;
      
      while(tibia_value < TIBIA_MAX){

	joint_state.header.stamp = ros::Time::now();
	//	ROS_INFO("%f",tibia_value);
	joint_state.position[0] = coxa_value;
	joint_state.position[6] = femur_value;
	joint_state.position[12] = tibia_value;
      
	joint_pub.publish(joint_state);
      
	ros::Time now = ros::Time(0);
	listener.waitForTransform("/perna_centro_1", "/tibia_foot_1", now, ros::Duration(1));
	listener.lookupTransform("/perna_centro_1", "/tibia_foot_1", 
				 now, transform);
      
	dataFile << joint_state.position[0] <<" "<< joint_state.position[6] << " "
		 << joint_state.position[12] << " " << transform.getOrigin().x() << " " 
		 <<transform.getOrigin().y() <<" " << transform.getOrigin().z() << "\n";

	tibia_value += increment;
	loop_rate.sleep();

      
	tibia_value +=increment;
      }
      femur_value += increment;
    }
    coxa_value += increment;
  }
  dataFile.close();
  
  return 0;
}
  

