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
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>

//
using namespace hrvo;
using namespace std;

const float HRVO_TWO_PI = 6.283185307179586f;
static string fixed_frame = "base_footprint";
//
ros::Publisher robot_body_pub, robot_velocity_vector_pub, ref_velocity_pub;
//
visualization_msgs::Marker createMarkerRobotBody(float_t x, float_t y, float_t z, int id_point){

    visualization_msgs::Marker marker;
    //
    std::stringstream ss;
    ss << "robot_body_hrvo";
    //
    marker.header.frame_id = fixed_frame;//world
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
    ss << "vector_velocity_hrvo";

    marker.header.frame_id = fixed_frame;//world
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
    marker.color.r = 0.5;
    marker.color.g = 0.5;
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
void agentsStatesCallback(const gazebo_msgs::ModelStatesConstPtr & msg){

    //std::cout << "Check agents state callback" << endl;

    Simulator simulator;
    visualization_msgs::Marker robotmarker, velocitymarker;
    visualization_msgs::MarkerArray robotmarkerarray, velocitymarkerarray;
    geometry_msgs::Twist ref_velocity;
    gazebo_msgs::ModelStates ref_velocity_array;

    simulator.setTimeStep(0.25f);
    // setAgentDefaults(float neighborDist, std::size_t maxNeighbors, float radius, float goalRadius, float prefSpeed, float maxSpeed)
    //simulator.setAgentDefaults(15.0f, 10, 2.0f, 0.5f, 1.0f, 1.5f);
    //std::size_t Simulator::addAgent(const Vector2 &position, std::size_t goalNo, float neighborDist, std::size_t maxNeighbors, float radius, float goalRadius, float prefSpeed, float maxSpeed,
    //float uncertaintyOffset, float maxAccel, const Vector2 &velocity, float orientation)
    // init the status of the agent
    for (std::size_t i = 0; i < msg->name.size(); ++i) {
        const Vector2 position = Vector2(msg->pose[i].position.x, msg->pose[i].position.y);
        const Vector2 goal = Vector2(msg->twist[i].angular.x, msg->twist[i].angular.y);
        const Vector2 vel = Vector2(msg->twist[i].linear.x, msg->twist[i].linear.y);
        float orientation = atan2(msg->twist[i].linear.y, msg->twist[i].linear.x);
        //ROS_INFO("Goal Position: %f, %f", msg->twist[i].angular.x, msg->twist[i].angular.y);
        //simulator.addAgent(position, simulator.addGoal(goal));
        simulator.addAgent(position,                // const Vector2 &position
                           simulator.addGoal(goal), // std::size_t goalNo
                           8.0f,                   // float neighborDist
                           15.0f,                      // std::size_t maxNeighbors
                           0.32f,                    // float radius 1.0
                           1.0f,                    // float goalRadius
                           1.0f,                    // float prefSpeed
                           1.5f,                    // float maxSpeed
                           0.0f,                    // float uncertaintyOffset
                           1.5f,                    // float maxAccel
                           vel,                     // const Vector2 &velocity
                           0.0f);                   // float orientation
    }
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
    robot_body_pub.publish(robotmarkerarray);
    robot_velocity_vector_pub.publish(velocitymarkerarray);
    ref_velocity_pub.publish(ref_velocity_array);
}
//
int main(int argc, char **argv)
{

    ros::init(argc, argv, "compute_ref_velocity_using_hrvo_real_experiment_nonepssm");
    ros::NodeHandle nh;
    ros::Subscriber agents_states_sub;
    // vis
    robot_body_pub = nh.advertise<visualization_msgs::MarkerArray>("hrvo/robot_body", 0);
    robot_velocity_vector_pub = nh.advertise<visualization_msgs::MarkerArray>("hrvo/robot_velocity_vector",0);
    // data
    ref_velocity_pub = nh.advertise<gazebo_msgs::ModelStates>("/hrvo/ref_velocity",1);
    //
    agents_states_sub = nh.subscribe("/pedsim/agents_states",5,&agentsStatesCallback);
    //

    ros::Rate rate(5);
    while (ros::ok())
    {


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}



