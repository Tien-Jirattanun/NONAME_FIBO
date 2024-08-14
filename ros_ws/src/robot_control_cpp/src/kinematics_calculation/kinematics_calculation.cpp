#include "kinematics_calculation.hpp"

KinematicsCalculation::KinematicsCalculation() : wheel_vel_(4, 0.0), swerve_pos_(4, 0.0) {}

void KinematicsCalculation::kinematics_calculation(double Vx, double Vy, double W, std::vector<std::vector<double>> tf_)
{

    // Linear queation only for translation

    // W = 0.0;
    // double V = std::sqrt(Vx * Vx + Vy * Vy);
    // double theta = std::atan2(Vy, Vx);

    // for (int i = 0; i < 4; ++i)
    // {
    //     wheel_vel_[i] = V;
    //     swerve_pos_[i] = theta;
    // }

    // Kinematiscs that calculate the robot rotation + translation

    std::vector<double> translation_velocity = {Vx, Vy, 0};
    std::vector<double> angular_velocity = {0, 0, W};

    for (int swerve_number = 0; swerve_number < 4; ++swerve_number)
    {
        std::vector<double> independent_tf = {tf_[swerve_number][0], tf_[swerve_number][1], 0.0};
        std::vector<double> output_vector = add_vector(translation_velocity, cross_vector(angular_velocity, independent_tf));

        // magnitude of the vector
        wheel_vel_[swerve_number] = sqrt(pow(output_vector[0], 2) + pow(output_vector[1], 2) + pow(output_vector[2], 2));
        swerve_pos_[swerve_number] = atan2(output_vector[1], output_vector[0]);
    }
}

std::vector<double> KinematicsCalculation::cross_vector(const std::vector<double> &A, const std::vector<double> &B)
{
    std::vector<double> C(3);

    C[0] = A[1] * B[2] - A[2] * B[1];
    C[1] = A[2] * B[0] - A[0] * B[2];
    C[2] = A[0] * B[1] - A[1] * B[0];

    return C;
}

std::vector<double> KinematicsCalculation::add_vector(const std::vector<double> &A, const std::vector<double> &B)
{
    std::vector<double> C(A.size());

    for (size_t i = 0; i < A.size(); ++i) {
        C[i] = A[i] + B[i];
    }

    return C;

}

const std::vector<double> &KinematicsCalculation::get_wheel_vel() const
{
    return wheel_vel_;
}

const std::vector<double> &KinematicsCalculation::get_swerve_pos() const
{
    return swerve_pos_;
}