/** It works well
 *  Change the NUMBER_OF_AGENTS value to get more agent
 *  Create a mobile robot
*/
#define HRVO_OUTPUT_TIME_AND_POSITIONS 1
#define NUMBER_OF_AGENTS 8

#include <cmath>
#include <iostream>
#include <hrvo/hrvo.h>
//
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
//
#include <std_msgs/String.h>
#include <hrvo/Clusters.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
//
 #include <tf/transform_broadcaster.h>
//
using namespace hrvo;
using namespace std;

static string fixed_frame = "laser";
//
double dt = 0.025;
ros::Publisher robot_pose_pub, robot_linearvel_pub, robot_goal_pub;
ros::Publisher robot_cadrl_pub, robot_cadrl_velocity_vector_pub;
geometry_msgs::Twist robot_twist_;
geometry_msgs::Pose robot_pose_;

//
visualization_msgs::Marker createMarkerRobotBody(double x, double y, double z, int id_point){

    visualization_msgs::Marker marker;
    //
    std::stringstream ss;
    ss << "robot_cadrl_simulation";
    //
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = ss.str();
    marker.id = id_point; // this number should be different
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    //marker.lifetime = ros::Duration(0.5); // the marker will be deleted after 0.5s
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 2*z;

    marker.color.a = 1;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;
    return marker;
}

visualization_msgs::Marker createVector(double xpos, double ypos, double zpos, double vx, double vy, int id_point)
{
    visualization_msgs::Marker marker;
    geometry_msgs::Point p;

    std::stringstream ss;
    ss << "vector_velocity_card_robot";

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = ss.str();
    marker.id = id_point; // this number should be different
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.12;
    marker.scale.z = 0.12;

    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    //
    p.x = xpos; p.y = ypos; p.z = zpos;
    marker.points.push_back(p);
    //
    p.x = xpos + vx; p.y = ypos + vy;  p.z = zpos;
    marker.points.push_back(p);

    return marker;
}
//
void kinematicModel(double vellinear, double velangular, double dt){


    double x = robot_pose_.position.x;
    double y = robot_pose_.position.y;
    double theta = robot_pose_.orientation.z;

    robot_pose_.position.x = x+vellinear*cos(theta)*dt;
    robot_pose_.position.y = y+vellinear*sin(theta)*dt;
    robot_pose_.orientation.z = theta+velangular*dt*5;
}
//
void robotVelocityCadrlCallback(const geometry_msgs::TwistConstPtr & msg){

    robot_twist_ = *msg;
    double vrlx = msg->linear.x;
    double vraz = msg->angular.z;
    ROS_INFO("RobotVelCadrl vx = %f, va = %f",vrlx,vraz);
    //
    kinematicModel(vrlx, vraz, dt);
    //
    visualization_msgs::Marker robotmarker, velocitymarker;
    //
    geometry_msgs::PoseStamped robot_pose;
    geometry_msgs::PoseStamped robot_goal;
    geometry_msgs::Vector3 robot_linearvel;
    std_msgs::Header robot_header;
    robot_header.frame_id = "world";
    robot_header.stamp = ros::Time::now();
    //
    double xpos = robot_pose_.position.x;
    double ypos = robot_pose_.position.y;
    double velx = robot_twist_.linear.x*cos(robot_pose_.orientation.z);
    double vely = robot_twist_.linear.x*sin(robot_pose_.orientation.z);
    robotmarker = createMarkerRobotBody(xpos,ypos,0.6,0);
    velocitymarker = createVector(xpos,ypos,0.6,velx,vely,0);
    //
    robot_pose.pose.position.x= xpos;
    robot_pose.pose.position.y= ypos;
    robot_pose.pose.position.z=0;
    robot_pose.header = robot_header;

    robot_goal.pose.position.x= robot_pose_.orientation.x;
    robot_goal.pose.position.y= robot_pose_.orientation.y;
    robot_goal.pose.position.z=0;
    robot_goal.header = robot_header;

    robot_linearvel.x =velx;
    robot_linearvel.y =vely;
    robot_linearvel.z =0;

    //double theta = atan2 (vely, velx);
    double theta = robot_pose_.orientation.z;
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(theta);
    robot_pose.pose.orientation=quat;
    //
    robot_cadrl_pub.publish(robotmarker);
    robot_cadrl_velocity_vector_pub.publish(velocitymarker);
    //
    robot_pose_pub.publish(robot_pose);
    robot_goal_pub.publish(robot_goal);
    robot_linearvel_pub.publish(robot_linearvel);
}
//
void robotNewGoalCallback(const geometry_msgs::PoseStampedConstPtr & msg){
    robot_pose_.orientation.x=msg->pose.position.x;
    robot_pose_.orientation.y=msg->pose.position.y;
}
//
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cadrl_robot_in_ros");
    ros::NodeHandle nh;
    //
    tf::TransformBroadcaster g_transformBroadcaster;

    robot_cadrl_pub = nh.advertise<visualization_msgs::Marker>("cadrlsim/robot_body", 0);
    robot_cadrl_velocity_vector_pub = nh.advertise<visualization_msgs::Marker>("cadrlsim/robot_velocity_vector",0);
    //
    ros::Subscriber robot_vel_cadrl_sub = nh.subscribe("/upei/nn_cmd_vel",0,&robotVelocityCadrlCallback);
    ros::Subscriber robot_goal_cadrl_sub = nh.subscribe("move_base_simple/goal",0,&robotNewGoalCallback);

    robot_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/upei/pose", 0);
    robot_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/upei/move_base_simple/goal",0);
    robot_linearvel_pub = nh.advertise<geometry_msgs::Vector3>("/upei/velocity",0);
    //
    robot_pose_.position.x=10;
    robot_pose_.position.y=0;
    robot_pose_.orientation.x=-10;
    robot_pose_.orientation.y=0;
    robot_pose_.orientation.z = atan2(0-0,-10-10);
    //
    robot_twist_.linear.x=0;
    robot_twist_.angular.z =0;
    //
    ros::Rate rate(100); //5
    while (ros::ok())
    {
        // Broadcast transform
        /*
        tf::Transform g_currentPose;
        g_currentPose.getOrigin().setX(robot_pose_.position.x);
        g_currentPose.getOrigin().setY(robot_pose_.position.y);
        double theta = robot_pose_.orientation.z;
        g_currentPose.setRotation( tf::createQuaternionFromRPY(0, 0, theta) );
        g_transformBroadcaster.sendTransform( tf::StampedTransform(g_currentPose, ros::Time::now(), "world", "base_footprint") );
        */
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
