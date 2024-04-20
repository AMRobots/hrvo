/** It works well
 *  Change the NUMBER_OF_AGENTS value to get more agent
*/
#define HRVO_OUTPUT_TIME_AND_POSITIONS 1
#define NUMBER_OF_AGENTS 10

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

const float HRVO_TWO_PI = 6.283185307179586f;
static string fixed_frame = "world";
//

ros::Publisher  clusters_pub;
//
visualization_msgs::Marker createMarkerRobotBody(double x, double y, double z, int id_point){

    visualization_msgs::Marker marker;
    //
    std::stringstream ss;
    ss << "robot_body_hrvo";
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
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 2*z;

    marker.color.a = 1;
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    return marker;
}

visualization_msgs::Marker createVector(double xpos, double ypos, double zpos, double vx, double vy, int id_point)
{
    visualization_msgs::Marker marker;
    geometry_msgs::Point p;

    std::stringstream ss;
    ss << "vector_velocity_hrvo";

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
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_hrvo_in_ros");
    ros::NodeHandle nh;
    ros::Publisher robot_body_pub, robot_velocity_vector_pub;
    //
    robot_body_pub = nh.advertise<visualization_msgs::MarkerArray>("hrvo/robot_body", 0);
    robot_velocity_vector_pub = nh.advertise<visualization_msgs::MarkerArray>("hrvo/robot_velocity_vector",0);
    clusters_pub = nh.advertise<ford_msgs::Clusters>("/upei/cluster/output/clusters",0);
    //
    Simulator simulator;

    simulator.setTimeStep(0.25f); // 0.25
    // setAgentDefaults(float neighborDist, std::size_t maxNeighbors, float radius, float goalRadius, float prefSpeed, float maxSpeed)
    simulator.setAgentDefaults(15.0f, 10, 0.25f, 1.0f, 1.0f, 2.0f);
    // init the status of the agent

     /* Three people are moving down */
    const Vector2 iposition0 = Vector2(1,-10); const Vector2 igoal0 = Vector2(1,10);
    simulator.addAgent(iposition0, simulator.addGoal(igoal0));
    const Vector2 iposition1 = Vector2(0,-10); const Vector2 igoal1 = Vector2(0,10);
    simulator.addAgent(iposition1, simulator.addGoal(igoal1));
    const Vector2 iposition2 = Vector2(-1,-10); const Vector2 igoal2 = Vector2(-1,10);
    simulator.addAgent(iposition2, simulator.addGoal(igoal2));

    /* Three moving people
    const Vector2 iposition0 = Vector2(-10,1); const Vector2 igoal0 = Vector2(10,1);
    simulator.addAgent(iposition0, simulator.addGoal(igoal0));
    const Vector2 iposition1 = Vector2(-10,0); const Vector2 igoal1 = Vector2(10,0);
    simulator.addAgent(iposition1, simulator.addGoal(igoal1));
    const Vector2 iposition2 = Vector2(-10,-1); const Vector2 igoal2 = Vector2(10,-1);
    simulator.addAgent(iposition2, simulator.addGoal(igoal2));
    */
    /* Five people are moving forward
    double iagents[4][NUMBER_OF_AGENTS]={{-8,-8,-8,-8,-8,-8,-8,-8},
                                       {4,  3, 3, 1, 1, -1, -3, -3},
                                       {8,8,8,8,8,8,8,8},
                                       {4,  3, 3, 1, 1, -1, -3, -3}};
    for (std::size_t i = 0; i < NUMBER_OF_AGENTS; ++i) {
        const Vector2 iposition = Vector2(iagents[0][i],iagents[1][i]);
        const Vector2 igoal = Vector2(iagents[2][i],iagents[3][i]);
        simulator.addAgent(iposition, simulator.addGoal(igoal));
    }
    */
    /* Eight moving people in circle
    for (std::size_t i = 1; i < NUMBER_OF_AGENTS; ++i) {
        const Vector2 position = NUMBER_OF_AGENTS * 1.0f * Vector2(std::cos(1.000f * i * HRVO_TWO_PI/NUMBER_OF_AGENTS), std::sin(1.000f * i * HRVO_TWO_PI/NUMBER_OF_AGENTS));
        simulator.addAgent(position, simulator.addGoal(-position));
    }
    */
    bool predictmode = true;
    ros::Rate rate(12); // 5, 7.5, 10
    while ((ros::ok())&(!simulator.haveReachedGoals()))
    {
        //std::cout << simulator.getGlobalTime(); // get global time
        visualization_msgs::Marker robotmarker, velocitymarker;
        visualization_msgs::MarkerArray robotmarkerarray, velocitymarkerarray;
        //
        std_msgs::Header robot_header;
        robot_header.frame_id = "world";
        robot_header.stamp = ros::Time::now();
        ford_msgs::Clusters clusters_agent;
        //
        for (std::size_t i = 0; i < simulator.getNumAgents(); ++i) {

            Vector2 pos = simulator.getAgentPosition(i);
            robotmarker = createMarkerRobotBody(pos.getX(),pos.getY(),0.6,i);
            robotmarkerarray.markers.push_back(robotmarker);
            Vector2 vel = simulator.getAgentVelocity(i);
            velocitymarker = createVector(pos.getX(),pos.getY(),0.6,vel.getX(),vel.getY(),i);
            velocitymarkerarray.markers.push_back(velocitymarker);
            //
            clusters_agent.header=robot_header;
            uint32_t counta;
            counta = i+1;
            clusters_agent.labels.push_back(counta);
            clusters_agent.counts.push_back(counta);
            geometry_msgs::Point agent_pos;
            Vector2 pos1 = simulator.getAgentPosition(i);
            agent_pos.x=pos1.getX();
            agent_pos.y=pos1.getY();
            agent_pos.z=0;
            clusters_agent.mean_points.push_back(agent_pos);
            clusters_agent.max_points.push_back(agent_pos);
            clusters_agent.min_points.push_back(agent_pos);
            geometry_msgs::Vector3 agent_vel;
            Vector2 vel1 = simulator.getAgentVelocity(i);
            agent_vel.x = vel1.getX();
            agent_vel.y = vel1.getY();
            agent_vel.z = 0;
            clusters_agent.velocities.push_back(agent_vel);
        }
        //
        uint32_t nagents = simulator.getNumAgents();
        if(predictmode){
            for (std::size_t i = 0; i < simulator.getNumAgents(); ++i) {

                clusters_agent.header=robot_header;
                uint32_t counta;
                counta = i+1+nagents;
                clusters_agent.labels.push_back(counta);
                clusters_agent.counts.push_back(counta);

                geometry_msgs::Vector3 agent_vel;
                Vector2 vel1 = simulator.getAgentVelocity(i);
                agent_vel.x = vel1.getX();
                agent_vel.y = vel1.getY();
                agent_vel.z = 0;
                clusters_agent.velocities.push_back(agent_vel);

                double delta_t = 0.25*5;
                geometry_msgs::Point agent_pos;
                Vector2 pos1 = simulator.getAgentPosition(i);
                agent_pos.x=pos1.getX()+vel1.getX()*delta_t;
                agent_pos.y=pos1.getY()+vel1.getY()*delta_t;
                agent_pos.z=0;
                clusters_agent.mean_points.push_back(agent_pos);
                clusters_agent.max_points.push_back(agent_pos);
                clusters_agent.min_points.push_back(agent_pos);
            }
        }
        //
        robot_body_pub.publish(robotmarkerarray);
        robot_velocity_vector_pub.publish(velocitymarkerarray);
        clusters_pub.publish(clusters_agent);

        simulator.doStep();

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
