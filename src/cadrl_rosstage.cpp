#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <hrvo/Clusters.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

ros::Publisher clusters_pub, cmdvel_robot_pub;
ros::Publisher robot_pose_pub, robot_linearvel_pub, robot_goal_pub;
ros::Publisher robot_cadrl_pub, robot_cadrl_velocity_vector_pub;
geometry_msgs::Twist robot_twist_;
geometry_msgs::Pose robot_pose_, goal_pose_;

uint32_t robotid = 0; uint32_t goalid = -1; //
string vel_robot_topic = "robot_0/cmd_vel";
//
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
void robotVelocityCadrlCallback(const geometry_msgs::TwistConstPtr & msg){

    robot_twist_ = *msg;
    double vrlx = msg->linear.x;
    double vraz = msg->angular.z;
    //ROS_INFO("RobotVelCadrl vx = %f, va = %f",vrlx,vraz);
    geometry_msgs::Twist twist_cadrl;
    twist_cadrl.linear.x  = vrlx;
    twist_cadrl.angular.z = vraz;
    cmdvel_robot_pub.publish(twist_cadrl);
}
//
void agentsStatesCallback(const gazebo_msgs::ModelStatesConstPtr & msg){

    geometry_msgs::PoseStamped robot_pose;
    geometry_msgs::PoseStamped robot_goal;
    geometry_msgs::Vector3 robot_linearvel;
    std_msgs::Header robot_header;
    robot_header.frame_id = "world";
    robot_header.stamp = ros::Time::now();

    ford_msgs::Clusters clusters_agent;
    //for (std::size_t i = 0; i < msg->name.size(); ++i) {
    for (uint32_t i = 0; i < msg->name.size(); ++i) {
        if(i !=robotid){
            if(i==goalid){
                robot_goal.pose.position.x= msg->pose[i].position.x;
                robot_goal.pose.position.y= msg->pose[i].position.y;
            }else{
                clusters_agent.header=robot_header;
                uint32_t counta;
                counta = i+1;
                //ROS_INFO("Agent %d name %s", counta, msg->name[i].c_str());
                clusters_agent.labels.push_back(counta);
                clusters_agent.counts.push_back(counta);
                geometry_msgs::Point agent_pos;
                agent_pos.x=msg->pose[i].position.x;
                agent_pos.y=msg->pose[i].position.y;
                agent_pos.z=0;
                clusters_agent.mean_points.push_back(agent_pos);
                clusters_agent.max_points.push_back(agent_pos);
                clusters_agent.min_points.push_back(agent_pos);
                geometry_msgs::Vector3 agent_vel;
                //agent_vel.x = msg->twist[i].linear.x;
                //agent_vel.y = msg->twist[i].linear.y;
                agent_vel.x = 0.0;
                agent_vel.y = 0.0;
                agent_vel.z = 0.0;
                clusters_agent.velocities.push_back(agent_vel);
            }
        }else{// robot
            robot_pose.header = robot_header;
            robot_pose.pose.position.x= msg->pose[i].position.x;
            robot_pose.pose.position.y= msg->pose[i].position.y;
            robot_pose.pose.position.z=0;
            robot_pose.pose.orientation = msg->pose[i].orientation;
            //robot_linearvel.x = msg->twist[i].linear.x;
            //robot_linearvel.y = msg->twist[i].linear.y;
            robot_linearvel.x = 0.0;
            robot_linearvel.y = 0.0;
            robot_linearvel.z = 0.0;
        }
    }
    robot_goal.pose.position.x= goal_pose_.position.x;
    robot_goal.pose.position.y= goal_pose_.position.y;
    robot_goal.pose.position.z=0;
    robot_goal.header = robot_header;

    clusters_pub.publish(clusters_agent);
    robot_linearvel_pub.publish(robot_linearvel);
    robot_pose_pub.publish(robot_pose);
    robot_goal_pub.publish(robot_goal);
}
//
void robotNewGoalCallback(const geometry_msgs::PoseStampedConstPtr & msg){
    goal_pose_.position.x=msg->pose.position.x;
    goal_pose_.position.y=msg->pose.position.y;
    ROS_INFO("Goal position: x= %f, y = %f", goal_pose_.position.x, goal_pose_.position.x);
}
//
int main(int argc, char **argv)
{

    ros::init(argc, argv, "cadrl_rosstage_node");
    ros::NodeHandle nh;
    ros::Subscriber agents_states_sub = nh.subscribe("/stage/agents_states", 1, agentsStatesCallback);
    ros::Subscriber robot_vel_cadrl_sub = nh.subscribe("/upei/nn_cmd_vel",1,&robotVelocityCadrlCallback);
    ros::Subscriber robot_goal_cadrl_sub = nh.subscribe("move_base_simple/goal",1,&robotNewGoalCallback);

    cmdvel_robot_pub = nh.advertise<geometry_msgs::Twist>(vel_robot_topic,1);

    clusters_pub = nh.advertise<ford_msgs::Clusters>("/upei/cluster/output/clusters",1);
    robot_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/upei/pose", 1);
    robot_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/upei/move_base_simple/goal",1);
    robot_linearvel_pub = nh.advertise<geometry_msgs::Vector3>("/upei/velocity",1);
    //
    goal_pose_.position.x=0.0;
    goal_pose_.position.y=0.0;
    //
    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    //ros::spin();
    return 0;
}

