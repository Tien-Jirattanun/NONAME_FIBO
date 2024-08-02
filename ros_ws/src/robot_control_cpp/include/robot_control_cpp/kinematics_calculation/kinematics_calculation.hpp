#ifndef KINEMATICS_CALCULATION_HPP
#define KINEMATICS_CALCULATION_HPP

#include <cmath>
#include <vector>

class KinematicsCalculation
{
public:
    KinematicsCalculation();

    void kinematics_calculation(double Vx, double Vy, double W);

    const std::vector<double>& get_wheel_vel() const;
    const std::vector<double>& get_swerve_pos() const;

private:
    std::vector<double> wheel_vel_;
    std::vector<double> swerve_pos_;
};

#endif // KINEMATICS_CALCULATION_H