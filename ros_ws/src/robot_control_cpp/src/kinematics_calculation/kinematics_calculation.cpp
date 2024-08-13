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


}

const std::vector<double> &KinematicsCalculation::get_wheel_vel() const
{
    return wheel_vel_;
}

const std::vector<double> &KinematicsCalculation::get_swerve_pos() const
{
    return swerve_pos_;
}