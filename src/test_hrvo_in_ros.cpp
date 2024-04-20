/** It works well
 *  Change the NUMBER_OF_AGENTS value to get more agent
*/
#define HRVO_OUTPUT_TIME_AND_POSITIONS 1
#define NUMBER_OF_AGENTS 16

#include <cmath>
#include <iostream>
#include <hrvo/hrvo.h>
//
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
//
using namespace hrvo;
using namespace std;

const float HRVO_TWO_PI = 6.283185307179586f;
static string fixed_frame = "laser";
//
//pub_agents_states_ = nh_.advertise<gazebo_msgs::ModelStates>("/pedsim/agents_states",5);
//
visualization_msgs::Marker createMarkerRobotBody(float_t x, float_t y, float_t z, int id_point){

    visualization_msgs::Marker marker;
    //
    std::stringstream ss;
    ss << "robot_body_hrvo";
    //
    marker.header.frame_id = "map";
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

    marker.header.frame_id = "map";
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

int main(int argc, char **argv)
{

    ros::init(argc, argv, "test_hrvo_in_ros");
    ros::NodeHandle nh;
    ros::Publisher robot_body_pub, robot_velocity_vector_pub;

    robot_body_pub = nh.advertise<visualization_msgs::MarkerArray>("hrvo/robot_body", 0);
    robot_velocity_vector_pub = nh.advertise<visualization_msgs::MarkerArray>("hrvo/robot_velocity_vector",0);

    Simulator simulator;

    simulator.setTimeStep(0.25f);
    // setAgentDefaults(float neighborDist, std::size_t maxNeighbors, float radius, float goalRadius, float prefSpeed, float maxSpeed)
    //simulator.setAgentDefaults(15.0f, 10, 1.5f, 1.5f, 1.0f, 2.0f);
    simulator.setAgentDefaults(15.0f, 10, 1.5f, 1.5f, 1.0f, 2.0f);
    // init the status of the agent
    for (std::size_t i = 0; i < NUMBER_OF_AGENTS; ++i) {
        const Vector2 position = NUMBER_OF_AGENTS * 1.0f * Vector2(std::cos(1.000f * i * HRVO_TWO_PI/NUMBER_OF_AGENTS), std::sin(1.000f * i * HRVO_TWO_PI/NUMBER_OF_AGENTS));
        simulator.addAgent(position, simulator.addGoal(-position));
    }

    ros::Rate rate(10); //5
    while ((ros::ok())&(!simulator.haveReachedGoals()))
    {
        std::cout << simulator.getGlobalTime(); // get global time
        visualization_msgs::Marker robotmarker, velocitymarker;
        visualization_msgs::MarkerArray robotmarkerarray, velocitymarkerarray;
        for (std::size_t i = 0; i < simulator.getNumAgents(); ++i) {
            //std::cout << " " << simulator.getAgentPosition(i);
            std::cout << "vref = " << simulator.getAgentPrefSpeed(1);
            std::cout << "vcur = " << simulator.getAgentVelocity(1);
            Vector2 pos = simulator.getAgentPosition(i);

            robotmarker = createMarkerRobotBody(pos.getX(),pos.getY(),0.6,i);
            robotmarkerarray.markers.push_back(robotmarker);

            Vector2 vel = simulator.getAgentVelocity(i);
            velocitymarker = createVector(pos.getX(),pos.getY(),0.6,vel.getX(),vel.getY(),i);
            velocitymarkerarray.markers.push_back(velocitymarker);
        }
        robot_body_pub.publish(robotmarkerarray);
        robot_velocity_vector_pub.publish(velocitymarkerarray);

        std::cout << std::endl;
        simulator.doStep();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
