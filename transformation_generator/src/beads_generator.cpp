#include <ros/ros.h>
#include <Eigen/Eigen>
#include <transformation_generator/transformation_generator.h>
#include <transformation_generator/ListOfPoints.h>


int main(int argc, char** argv) {
	ros::init(argc, argv, "beads_generator"); //node name
	ros::NodeHandle nh; // don't really need this in this example
	TransformationGenerator beadsGenerator(&nh);

	transformation_generator::ListOfPoints list_of_points;
	beadsGenerator.getBeadsPosition(9, 3, 3, list_of_points);

	ros::spin();
	return 0;
}
