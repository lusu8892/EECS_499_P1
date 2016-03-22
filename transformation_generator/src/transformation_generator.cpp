// transformation_generator
//

#include <transformation_generator/transformation_generator.h>

const double BEADS_SEPERATION_VALUE = 0.0254;

// Constructor
// TransformationGenerator::TransformationGenerator(ros::NodeHandle* nodehandle) : nh_(*nodehandle)
// {
// 	// initializePublishers();
// }

TransformationGenerator::TransformationGenerator()
{
	// initializePublishers();
}
// given randomly generated transformation matrix, this function can give a list of
// beads position
void TransformationGenerator::getBeadsPosition(int beads_number, int row_num, int col_num, transformation_generator::ListOfPoints& list_of_beads_pos)
{
	list_of_beads_pos.points.clear(); // clear vector;
	geometry_msgs::Point bead_position;

	Eigen::Vector3d Ob;
	Eigen::Affine3d random_trans_mat;
	random_trans_mat = randomTransformationMatrixGenerator();
	Eigen::Vector3d Oe = random_trans_mat.translation();
	Eigen::Vector3d beads_in_sensor_frame;

	for (int i = 0; i < row_num; ++i)
	{	
		Ob(0)= BEADS_SEPERATION_VALUE * i; // start from first row
		for (int j = 0; j < col_num; ++j)
		{
			Ob(1)= BEADS_SEPERATION_VALUE * j; // start from first colu
			Ob(2)= Oe(2);
			beads_in_sensor_frame = random_trans_mat * Ob;
			bead_position.x = beads_in_sensor_frame(0);
			bead_position.y = beads_in_sensor_frame(1);
			bead_position.z = beads_in_sensor_frame(2);
			list_of_beads_pos.points.push_back(bead_position);
		}
	}
	// beads_pos_pub_.publish(list_of_beads_pos);
}

// void TransformationGenerator::initializePublishers()
// {
// 	ROS_INFO("Initializing Publishers");
// 	beads_pos_pub_ = nh_.advertise<transformation_generator::ListOfPoints>("beads_random_position", 1, true);
// 	//add more publishers, as needed
// 	// note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
// }

// A function generate random number
double TransformationGenerator::gaussianNumberGenerator(double LO, double HI)
{
	double gaussian_num;
	int iteration = 5;
	// double LO = -0.01;
	// double HI = 0.01;

	for (int i = 0; i < iteration; ++i)
	{
		gaussian_num += LO + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(HI-LO))); 
	}
	return gaussian_num;
}
// A function randomly generate transformation matrix
Eigen::Affine3d TransformationGenerator::randomTransformationMatrixGenerator()
{
	Eigen::Affine3d random_trans_mat;
	Eigen::Vector3d Oe;
	Oe(0)= gaussianNumberGenerator(-0.01, 0.01);
	Oe(1)= gaussianNumberGenerator(-0.01, 0.01);
	Oe(2)= gaussianNumberGenerator(-0.01, 0.01);
	random_trans_mat.translation() = Oe; // the "translation" part of affine is the vector between origins
	Eigen::Quaterniond q;
	// Eigen::Quaterniond<Scalar> 
	// double magnitude;
	q.x() = gaussianNumberGenerator(-1, 1);
	q.y() = gaussianNumberGenerator(-1, 1);
	q.z() = gaussianNumberGenerator(-1, 1);
	q.w() = gaussianNumberGenerator(-1, 1);
	// magnitude = sart(q.x()^2 + q.y()^2 + q.z()^2 + q.w()^2);
	// // normalized q.x, q.y, q.z, q.w
	// q.x() = q.x() / magnitude;
	// q.y() = q.y() / magnitude;
	// q.z() = q.z() / magnitude;
	// q.w() = q.w() / magnitude;
	Eigen::Matrix3d Re(q.normalized()); //convenient conversion...initialize a 3x3 orientation matrix
	// using a quaternion, q
	random_trans_mat.linear() = Re; // the rotational part of affine is the "linear" part
	return random_trans_mat;
}
