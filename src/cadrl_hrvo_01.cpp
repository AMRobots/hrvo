/** It works well
 *  Change the NUMBER_OF_AGENTS value to get more agent
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

const float HRVO_TWO_PI = 6.283185307179586f;
static string fixed_frame = "laser";
//
ros::Publisher robot_pose_pub, robot_linearvel_pub, robot_goal_pub, clusters_pub;
geometry_msgs::Twist robot_twist_;
geometry_msgs::Pose robotpose_;
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
void robotVelocityCadrlCallback(const geometry_msgs::TwistConstPtr & msg){
    robot_twist_ = *msg;
    double vrlx = msg->linear.x;
    double vrly = msg->linear.y;
    double vraz = msg->angular.z;
    ROS_INFO("RobotVelCadrl vx = %f, vy = %f, va = %f",vrlx,vrly,vraz);
}
geometry_msgs::Pose kinematicModel(geometry_msgs::Pose pos, double dt){

    geometry_msgs::Pose robotpose;
    double vellinear = robot_twist_.linear.x;
    double velangular = robot_twist_.angular.z;
    double x = pos.position.x;
    double y = pos.position.y;
    double theta = pos.orientation.z;

    robotpose.position.x = x+vellinear*cos(theta)*dt;
    robotpose.position.y = y+vellinear*sin(theta)*dt;
    robotpose.orientation.z = theta+velangular*dt;

    return robotpose;
}
//


int main(int argc, char **argv)
{

    ros::init(argc, argv, "test_hrvo_in_ros");
    ros::NodeHandle nh;
    ros::Publisher robot_body_pub, robot_velocity_vector_pub;
    //
    ros::Subscriber robot_vel_cadrl_sub = nh.subscribe("/nn_cmd_vel",0,&robotVelocityCadrlCallback);
    //
    robot_body_pub = nh.advertise<visualization_msgs::MarkerArray>("hrvo/robot_body", 0);
    robot_velocity_vector_pub = nh.advertise<visualization_msgs::MarkerArray>("hrvo/robot_velocity_vector",0);
    //
    robot_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/pose", 0);
    robot_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",0);
    //robot_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",0);
    robot_linearvel_pub = nh.advertise<geometry_msgs::Vector3>("/velocity",0);
    clusters_pub = nh.advertise<ford_msgs::Clusters>("/cluster/output/clusters",0);
    //
    Simulator simulator;

    simulator.setTimeStep(0.25f);
    // setAgentDefaults(float neighborDist, std::size_t maxNeighbors, float radius, float goalRadius, float prefSpeed, float maxSpeed)
    simulator.setAgentDefaults(15.0f, 10, 0.25f, 1.0f, 1.0f, 2.0f);
    // init the status of the agent
    for (std::size_t i = 0; i < NUMBER_OF_AGENTS; ++i) {
        const Vector2 position = NUMBER_OF_AGENTS * 1.0f * Vector2(std::cos(1.000f * i * HRVO_TWO_PI/NUMBER_OF_AGENTS), std::sin(1.000f * i * HRVO_TWO_PI/NUMBER_OF_AGENTS));
        simulator.addAgent(position, simulator.addGoal(-position));
    }

    ros::Rate rate(5); //5
    while ((ros::ok())&(!simulator.haveReachedGoals()))
    {
        //std::cout << simulator.getGlobalTime(); // get global time
        visualization_msgs::Marker robotmarker, velocitymarker;
        visualization_msgs::MarkerArray robotmarkerarray, velocitymarkerarray;
        //
        geometry_msgs::PoseStamped robot_pose;
        geometry_msgs::PoseStamped robot_goal;
        geometry_msgs::Vector3 robot_linearvel;
        std_msgs::Header robot_header;
        robot_header.frame_id = "world";
        robot_header.stamp = ros::Time::now();
        ford_msgs::Clusters clusters_agent;
        //
        Vector2 posr, velr;
        //
        for (std::size_t i = 0; i < simulator.getNumAgents(); ++i) {
            //std::cout << " " << simulator.getAgentPosition(i);
            //std::cout << "vref = " << simulator.getAgentPrefSpeed(1);
            //std::cout << "vcur = " << simulator.getAgentVelocity(1);
            Vector2 pos = simulator.getAgentPosition(i);
            robotmarker = createMarkerRobotBody(pos.getX(),pos.getY(),0.6,i);
            robotmarkerarray.markers.push_back(robotmarker);
            Vector2 vel = simulator.getAgentVelocity(i);
            velocitymarker = createVector(pos.getX(),pos.getY(),0.6,vel.getX(),vel.getY(),i);
            velocitymarkerarray.markers.push_back(velocitymarker);
            //
            //velr = simulator.getAgentVelocity(i);
            //posr = simulator.getAgentPosition(i);
            if(i==0){
                //Vector2 velr0; velr0.setX(0); velr0.setY(0);
                //simulator.setAgentVelocity(i,velr0);
                //velr = simulator.getAgentVelocity(i);
                posr = simulator.getAgentPosition(i);
                //ROS_INFO("Agent velocity vx=%f, vy=%f",velr.getX(),velr.getY());

                Vector2 pos0 = simulator.getAgentPosition(i);
                robot_pose.pose.position.x= pos0.getX();
                robot_pose.pose.position.y= pos0.getY();
                robot_pose.pose.position.z=0;
                robot_pose.header = robot_header;

                Vector2 goal0 = simulator.getGoalPosition(i);
                robot_goal.pose.position.x= goal0.getX();
                robot_goal.pose.position.y= goal0.getY();
                robot_goal.pose.position.z=0;
                robot_goal.header = robot_header;

                Vector2 vel0 = simulator.getAgentVelocity(i);
                robot_linearvel.x =vel0.getX();
                robot_linearvel.y =vel0.getY();
                robot_linearvel.z =0;

                float theta = atan2 (vel0.getY(), vel0.getX());
                geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(theta);
                robot_pose.pose.orientation=quat;

            }else{
                clusters_agent.header=robot_header;
                uint32_t counta;
                counta = i;
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
        }
        //
        robot_body_pub.publish(robotmarkerarray);
        robot_velocity_vector_pub.publish(velocitymarkerarray);
        //
        robot_pose_pub.publish(robot_pose);
        robot_goal_pub.publish(robot_goal);
        robot_linearvel_pub.publish(robot_linearvel);
        clusters_pub.publish(clusters_agent);
        //
        //std::cout << std::endl;
        simulator.doStep();
        /*
        robotpose_ = kinematicModel(robotpose_, 0.25);
        for (std::size_t i = 0; i < simulator.getNumAgents(); ++i) {
            if(i==0){
                Vector2 tmp;
                tmp.setX(robotpose_.position.x);
                tmp.setY(robotpose_.position.y);
                simulator.setAgentPosition(i,tmp);
                Vector2 tmpvel;
                tmpvel.setX(robot_twist_.linear.x*cos(robotpose_.orientation.z));
                tmpvel.setY(robot_twist_.linear.x*sin(robotpose_.orientation.z));
                simulator.setAgentVelocity(i,tmpvel);
            }
        }
        */
        /*
        if((simulator.haveReachedGoals())){
           for (std::size_t i = 0; i < NUMBER_OF_AGENTS; ++i) {
                const Vector2 position = NUMBER_OF_AGENTS * 1.0f * Vector2(std::cos(1.000f * i * HRVO_TWO_PI/NUMBER_OF_AGENTS), std::sin(1.000f * i * HRVO_TWO_PI/NUMBER_OF_AGENTS));
                simulator.addAgent(position, simulator.addGoal(-position));
            }
        }
        */
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
