/** It works well
 *  Get the information of the agent from ped_sim
 *  Calculate the velocity
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
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <angles/angles.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//
using namespace hrvo;
using namespace std;

const float HRVO_TWO_PI = 6.283185307179586f;
static string fixed_frame = "laser";
//
ros::Publisher robot_body_pub, robot_velocity_vector_pub, ref_velocity_pub;
ros::Publisher cmdvel_h60_pub, cmdvel_h61_pub;
//
visualization_msgs::Marker createMarkerRobotBody(float_t x, float_t y, float_t z, int id_point){

    visualization_msgs::Marker marker;
    //
    std::stringstream ss;
    ss << "robot_body_control_humans_objects";
    //
    marker.header.frame_id = "map";//world
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
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 2*z;

    marker.color.a = 1;
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    return marker;
}

visualization_msgs::Marker createVector(float_t xpos, float_t ypos, float_t zpos, float_t vx, float_t vy, int id_point)
{
    visualization_msgs::Marker marker;
    geometry_msgs::Point p;

    std::stringstream ss;
    ss << "vector_velocity_control_humans_objects";

    marker.header.frame_id = "map";//world
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
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    //
    p.x = xpos; p.y = ypos; p.z = zpos;
    marker.points.push_back(p);
    //
    p.x = xpos + vx; p.y = ypos + vy;  p.z = zpos;
    marker.points.push_back(p);

    return marker;
}
//
/// compute control command
geometry_msgs::Twist computeControlCommand(Vector2 vel, geometry_msgs::Pose pose){

    geometry_msgs::Twist cmd_vel;
    double vlinear1 = 1.0* sqrt(vel.getX()*vel.getX()+vel.getY()*vel.getY());
    cmd_vel.linear.x = vlinear1;
    double theta_cur = tf::getYaw(pose.orientation);
    double theta_new = atan2(vel.getY(), vel.getX());
    // Convert to 0-2pi;
    theta_cur = angles::normalize_angle(theta_cur);
    theta_new = angles::normalize_angle(theta_new);
    //double diff = angles::shortest_angular_distance(theta_new, theta_cur);
    double diff = theta_new - theta_cur;
    double omega = 0;
    omega = diff/(1.8*1);//1.8*1
    cmd_vel.angular.z = omega;
    return cmd_vel;
}
//
void agentsStatesCallback(const gazebo_msgs::ModelStatesConstPtr & msg){

    Simulator simulator;
    visualization_msgs::Marker robotmarker, velocitymarker;
    visualization_msgs::MarkerArray robotmarkerarray, velocitymarkerarray;
    geometry_msgs::Twist ref_velocity;
    gazebo_msgs::ModelStates ref_velocity_array;

    simulator.setTimeStep(0.25f);
    for (std::size_t i = 12; i < msg->name.size(); ++i) {
        const Vector2 position = Vector2(msg->pose[i].position.x, msg->pose[i].position.y);
        const Vector2 goal = Vector2(msg->twist[i].angular.x, msg->twist[i].angular.y);
        //const Vector2 vel = Vector2(msg->twist[i].linear.x, msg->twist[i].linear.y);
        const Vector2 vel = Vector2(0, 0);
        float orientation = atan2(msg->twist[i].linear.y, msg->twist[i].linear.x);
        //ROS_INFO("Goal Position: %f, %f", msg->twist[i].angular.x, msg->twist[i].angular.y);
        //simulator.addAgent(position, simulator.addGoal(goal));
        simulator.addAgent(position,                // const Vector2 &position
                           simulator.addGoal(goal), // std::size_t goalNo
                           8.0f,                   // float neighborDist
                           15.0f,                      // std::size_t maxNeighbors
                           0.5f,                    // float radius 1.0
                           1.0f,                    // float goalRadius
                           1.0f,                    // float prefSpeed
                           1.5f,                    // float maxSpeed
                           0.0f,                    // float uncertaintyOffset
                           1.0f,                    // float maxAccel
                           vel,                     // const Vector2 &velocity
                           0.0f);                   // float orientation
    }
    // add objects
    //const Vector2 position1 = Vector2(13, 15); const Vector2 goal1 = Vector2(13, 15);const Vector2 vel1 = Vector2(0, 0);
    //simulator.addAgent(position1,simulator.addGoal(goal1),8.0f,15.0f,0.5f,1.0f,1.0f,1.5f, 0.0f,1.5f,vel1,0.0f);
    //const Vector2 position2 = Vector2(14, 15); const Vector2 goal2 = Vector2(14, 15);const Vector2 vel2 = Vector2(0, 0);
    //simulator.addAgent(position2,simulator.addGoal(goal2),8.0f,15.0f,0.5f,1.0f,1.0f,1.5f, 0.0f,1.5f,vel2,0.0f);
    //
    simulator.doStep();
    for (std::size_t i = 0; i < simulator.getNumAgents(); ++i) {  // general case
        //std::cout << " " << simulator.getAgentPosition(i);
        //std::cout << "vref = " << simulator.getAgentPrefSpeed(1);
        //std::cout << "vcur = " << simulator.getAgentVelocity(1);
        Vector2 pos = simulator.getAgentPosition(i);
        robotmarker = createMarkerRobotBody(pos.getX(),pos.getY(),0.6,i);
        robotmarkerarray.markers.push_back(robotmarker);
        Vector2 vel = simulator.getAgentVelocity(i);
        velocitymarker = createVector(pos.getX(),pos.getY(),1.2,vel.getX(),vel.getY(),i);
        velocitymarkerarray.markers.push_back(velocitymarker);
        ref_velocity.linear.x = vel.getX();
        ref_velocity.linear.y = vel.getY();
        ref_velocity_array.twist.push_back(ref_velocity);
    }
    //
    geometry_msgs::Twist vel_h60, vel_h61;
    geometry_msgs::Pose pose60 = msg->pose[60];
    Vector2 vel60 = simulator.getAgentVelocity(60-12);
    vel_h60 = computeControlCommand(vel60, pose60);

    cmdvel_h60_pub.publish(vel_h60);
    cmdvel_h61_pub.publish(vel_h61);

    //
    robot_body_pub.publish(robotmarkerarray);
    robot_velocity_vector_pub.publish(velocitymarkerarray);
    ref_velocity_pub.publish(ref_velocity_array);
}
//
int main(int argc, char **argv)
{

    ros::init(argc, argv, "compute_ref_velocity_using_hrvo");
    ros::NodeHandle nh;
    ros::Subscriber agents_states_sub;
    // vis
    robot_body_pub = nh.advertise<visualization_msgs::MarkerArray>("hrvo/robot_body", 0);
    robot_velocity_vector_pub = nh.advertise<visualization_msgs::MarkerArray>("hrvo/robot_velocity_vector",0);
    // data
    ref_velocity_pub = nh.advertise<gazebo_msgs::ModelStates>("/hrvo/ref_velocity",1);
    //
    cmdvel_h60_pub = nh.advertise<geometry_msgs::Twist>("robot_60/cmd_vel",1);
    cmdvel_h61_pub = nh.advertise<geometry_msgs::Twist>("robot_61/cmd_vel",1);
    //
    agents_states_sub = nh.subscribe("/stage/agents_states", 5, &agentsStatesCallback);
    //

    ros::Rate rate(5);
    while (ros::ok())
    {


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

