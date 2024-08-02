#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "kinematics_calculation.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class KinematicsPublisher : public rclcpp::Node
{

public:
	KinematicsPublisher() : Node("kinematics_publisher"), Vx_(0.0), Vy_(0.0), W_(0.0)
	{
		pub_pos = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);
		pub_vel = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_velocity_controller/commands", 10);
		sub_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&KinematicsPublisher::cmdvel_callback, this, _1));
		timer_ = this->create_wall_timer(500ms, std::bind(&KinematicsPublisher::timer_callback, this));
	}

private:
	void timer_callback()
	{
		auto vel_msg = std_msgs::msg::Float64MultiArray();
		auto pos_msg = std_msgs::msg::Float64MultiArray();
		vel_msg.data = wheel_vel_;
		pos_msg.data = swerve_pos_;
		pub_vel->publish(vel_msg);
		pub_pos->publish(pos_msg);
	}

	void cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
	{
		Vx_ = msg->linear.x;
		Vy_ = msg->linear.y;
		W_ = msg->angular.z;

		kinematics.kinematics_calculation(Vx_, Vy_, W_);

		wheel_vel_ = kinematics.get_wheel_vel();
		swerve_pos_ = kinematics.get_swerve_pos();
	}

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pos;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_vel;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;
	size_t count_;
	float_t Vx_, Vy_, W_;
	std::vector<double> wheel_vel_ = {0.0, 0.0, 0.0, 0.0};
	std::vector<double> swerve_pos_ = {0.0, 0.0, 0.0, 0.0};
	KinematicsCalculation kinematics;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<KinematicsPublisher>());
	rclcpp::shutdown();
	return 0;
}