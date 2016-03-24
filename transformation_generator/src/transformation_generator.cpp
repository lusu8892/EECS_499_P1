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
    Eigen::Matrix3d Re(q.normalized()); //convenient conversion...initialize a 3x3 orientation matrix
    // using a quaternion, q
    random_trans_mat.linear() = Re; // the rotational part of affine is the "linear" part
    return random_trans_mat;
}

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

Eigen::Affine3d TransformationGenerator::getExpoMatrix(velo_vec::velocityVector rand_body_velo, double delta_time, Eigen::Affine3d trans_mat)
{
    Eigen::Affine3d expo_mat;
    Eigen::Vector3d rot_omega;
    Eigen::Vector3d trans_velo;
    Eigen::Matrix3d 3_by_3_I; // 3-by-3 identity matrix

    3_by_3_I << << Eigen::MatrixXd::Identity(3,3);

    Eigen::Matrix3d skew_trans_mat = getSkewSymMatrix(trans_mat.translation());


    velo_vec::velocityVector rand_space_velo; // define a random space velocity

    // convert rand_body_velocity to spatial velocity
    rand_space_velo.tranV = trans_mat.linear() * rand_body_velo.transV + skew_trans_mat * trans_mat.linear() * rand_body_velo.angV;
    rand_space_velo.angV = trans_mat.linear() * rand_body_velo.angV;

    rot_omega = delta_time * rand_space_velo.tranV;
    trans_velo = delta_time * rand_space_velo.angV;

    if (rot_omega.norm() < 10e-5)
    {
        expo_mat.linear() = 3_by_3_I // set linear part as 3-by-3 matrix
        expo_mat.translation() = trans_velo;
    }
    else
    {
        double theta = rot_omega.norm()
        trans_velo /= theta;
        rot_omega /= theta;

        Eigen::Matrix3d skew_rot_omega = getSkewSymMatrix(rot_omega);
        
        expo_mat.linear() = 3_by_3_I + skew_rot_omega * sin(theta) + 
                            skew_rot_omega * skew_rot_omega * (1 - cos(theta));
        expo_mat.translation() = (3_by_3_I - expo_mat.linear()) * rot_omega.cross(trans_velo) +
                                rot_omega * rot_omega.transpose() * trans_velo * theta;
    }
    return expo_mat;
}

Eigen::Matrix3d TransformationGenerator::getSkewSymMatrix(Eigen::Vector3d vector_in)
{
    Eigen::Matrix3d t_hat;
    t_hat << 0, -vector_in(2), vector_in(1),
        vector_in(2), 0, -vector_in(0),
        -vector_in(1), vector_in(0), 0;

    return t_hat;
}

