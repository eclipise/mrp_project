#include <cmath>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int16.h>
#include <nav_msgs/Odometry.h>

ros::Publisher odom_pub;
nav_msgs::Odometry odomOld;
nav_msgs::Odometry odomNew;

const float initialX = 0.0;
const float initialY = 0.0;
const float initialTheta = 0.0;
const float PI = 3.14159;

const int TICKS_PER_REVOLUTION = 40;
const float WHEEL_RADIUS = 3.0; // inches
const float WHEEL_CIRCUM = 2 * PI * WHEEL_RADIUS;
const int GEARBOX_RATIO = 15;
// TODO: diameter of circle robot traverses when turning in place (distance from
// center of left wheels to center of right wheels)
const float WHEEL_BASE = 0;

float distanceLeft = 0;
float distanceRight = 0;

void calc_left_distance(const std_msgs::Int16 &FL_ticks) {
    static int prevTicks = 0;

    // Detects a tick over- or underflow by checking if the number of ticks has
    // changed by an unrealistic value (5000 ticks is ~200 feet). If that
    // happened during this interval, we ignore it and assume that the velocity
    // is unchanged.
    if (abs(prevTicks - FL_ticks.data) > 5000) {
        prevTicks = FL_ticks.data;
        return;
    }

    // Ticks since last measurement
    int ticks = abs(FL_ticks.data - prevTicks);

    float rotations = ticks / (float)TICKS_PER_REVOLUTION;

    distanceLeft = rotations * WHEEL_CIRCUM;
}

void calc_right_distance(const std_msgs::Int16 &FR_ticks) {
    static int prevTicks = 0;

    // Detects a tick over- or underflow by checking if the number of ticks has
    // changed by an unrealistic value (5000 ticks is ~200 feet). If that
    // happened during this interval, we ignore it and assume that the velocity
    // is unchanged.
    if (abs(prevTicks - FR_ticks.data) > 5000) {
        prevTicks = FR_ticks.data;
        return;
    }

    // Ticks since last measurement
    int ticks = abs(FR_ticks.data - prevTicks);

    float rotations = ticks / (float)TICKS_PER_REVOLUTION;

    distanceRight = rotations * WHEEL_CIRCUM;
}

void update_odom() {
    // Average distance traveled by the two sides
    double cycleDistance = (distanceRight + distanceLeft) / 2;

    // TODO: This formula may be wrong
    double cycleAngle = asin((distanceRight - distanceLeft) / WHEEL_BASE);

    // Position calculation
    odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + cycleDistance;
    odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + cycleDistance;
    odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z + cycleAngle;

    // Constrains the current angle to +-2PI
    if (odomNew.pose.pose.orientation.z > PI) {
        odomNew.pose.pose.orientation.z -= 2 * PI;
    } else if (odomNew.pose.pose.orientation.z < -PI) {
        odomNew.pose.pose.orientation.z += 2 * PI;
    }

    // Velocity calculation
    odomNew.header.stamp = ros::Time::now();
    odomNew.twist.twist.linear.x = cycleDistance / (odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());
    odomNew.twist.twist.angular.z = cycleAngle / (odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());

    // Preserve the current data for the next calculation
    odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
    odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
    odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z;
    odomOld.header.stamp = odomNew.header.stamp;

    odom_pub.publish(odomNew);
}

int main(int argc, char* argv[]) {

    odomNew.header.frame_id = "odom";
    odomNew.pose.pose.position.z = 0;
    odomNew.pose.pose.orientation.x = 0;
    odomNew.pose.pose.orientation.y = 0;
    odomNew.twist.twist.linear.x = 0;
    odomNew.twist.twist.linear.y = 0;
    odomNew.twist.twist.linear.z = 0;
    odomNew.twist.twist.angular.x = 0;
    odomNew.twist.twist.angular.y = 0;
    odomNew.twist.twist.angular.z = 0;
    
    odomOld.pose.pose.position.x = initialX;
    odomOld.pose.pose.position.y = initialY;
    odomOld.pose.pose.orientation.z = initialTheta;

    ros::init(argc, argv, "odometer_node");

    ros::NodeHandle nh;
    tf::TransformBroadcaster trans_broadcaster;

    ros::Subscriber FL_TickSub = nh.subscribe("FL_ticks", 100, calc_left_distance, ros::TransportHints().tcpNoDelay());
    ros::Subscriber FR_TickSub = nh.subscribe("FR_ticks", 100, calc_right_distance, ros::TransportHints().tcpNoDelay());

    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);

    // This should be set to at least double the encoder poll rate on the
    // Arduino to ensure that all measurements are performed on fresh data.
    ros::Rate rate(0.25); // measured in seconds (4hz) 

    while (ros::ok()) {
        update_odom();

        ros::spinOnce();
        rate.sleep();
    }

    /*
    // TODO: dummy values, replace with actual velocities calculated from ticks
    double vx = 0.1;
    double vy = -0.1;
    double vth = 0.1;

    while (nh.ok()) {
        ros::spinOnce();

        current_time = ros::Time::now();

        double dt =(current_time - prev_time).toSec();
        double dx = (vx * cos(th) - vy * sin(th)) * dt;
        double dy = (vx * sin(th) + vy * cos(th)) * dt;
        double dth = vth * dt;

        x += dx;
        y += dy;
        th += dth;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        trans_broadcaster.sendTransform(odom_trans);

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        odom_pub.publish(odom);

        prev_time = current_time;
        r.sleep();
    }
    */
}