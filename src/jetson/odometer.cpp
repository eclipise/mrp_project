#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <nav_msgs/Odometry.h>

ros::Publisher odom_pub;
nav_msgs::Odometry odomNew;
nav_msgs::Odometry odomOld;

const double initialX = 0.0;
const double initialY = 0.0;
const double initialTheta = 0.0;
const double PI = 3.14159;

const int TICKS_PER_REVOLUTION = 40;
const float WHEEL_RADIUS = 3.0; // inches
const int GEARBOX_RATIO = 15;

// const double WHEEL_BASE = 0;

void calc_left(const std_msgs::UInt16 &FL_ticks) {

}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "odometer_node");

    ros::NodeHandle nh;

    ros::Subscriber FL_TickSub = nh.subscribe("FL_ticks", 100, calc_left, ros::TransportHints().tcpNoDelay());
    ros::Subscriber FR_TickSub = nh.subscribe("FR_ticks", 100, calc_right, ros::TransportHints().tcpNoDelay());
}