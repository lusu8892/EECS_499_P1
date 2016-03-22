#include <ros/ros.h>
#include <Eigen/Eigen>
#include <transformation_generator/transformation_generator.h>
#include <transformation_generator/ListOfPoints.h>


int main(int argc, char** argv) {
	ros::init(argc, argv, "beads_generator"); //node name
	ros::NodeHandle nh; // don't really need this in this example
	ros::Publisher beads_pos_pub = nh.advertise<transformation_generator::ListOfPoints>("beads_random_position", 1, true);
	TransformationGenerator beadsGenerator();
	transformation_generator::ListOfPoints list_of_points;

	while(ros::ok())
	{
		beadsGenerator.getBeadsPosition(9, 3, 3, list_of_points);
		beads_pos_pub.publish(list_of_points);
		ros::Duration(1.0).sleep();
	}

	return 0;
}
