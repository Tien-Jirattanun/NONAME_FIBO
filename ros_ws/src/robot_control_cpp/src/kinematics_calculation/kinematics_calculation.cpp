#include "kinematics_calculation.hpp"

KinematicsCalculation::KinematicsCalculation() : wheel_vel_(4, 0.0), swerve_pos_(4, 0.0) {}

void KinematicsCalculation::kinematics_calculation(double Vx, double Vy, double W)
{
    W = 0.0;
    double V = std::sqrt(Vx * Vx + Vy * Vy);
    double theta = 0.0;

    if (Vy == 0)
    {
        if (Vx > 0)
        {
            theta = M_PI / 2;
        }
        else if (Vx < 0)
        {
            theta = -M_PI / 2;
        }
    }
    else
    {
        theta = std::atan2(Vx, Vy);
    }

    for (int i = 0; i < 4; ++i)
    {
        wheel_vel_[i] = V;
        swerve_pos_[i] = theta;
    }
}

const std::vector<double> &KinematicsCalculation::get_wheel_vel() const
{
    return wheel_vel_;
}

const std::vector<double> &KinematicsCalculation::get_swerve_pos() const
{
    return swerve_pos_;
}