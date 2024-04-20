#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <hrvo/Clusters.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>

using namespace std;

ros::Publisher clusters_pub;
//
void agentsStatesCallback(const gazebo_msgs::ModelStatesConstPtr & msg){

    std_msgs::Header robot_header;
    robot_header.frame_id = "world";
    robot_header.stamp = ros::Time::now();
    ford_msgs::Clusters clusters_agent;
    //for (std::size_t i = 0; i < msg->name.size(); ++i) {
    for (uint32_t i = 0; i < msg->name.size(); ++i) {
            clusters_agent.header=robot_header;
            uint32_t counta;
            counta = i;
            ROS_INFO("Agent %d name %s", counta, msg->name[i].c_str());
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
            agent_vel.x = msg->twist[i].linear.x;
            agent_vel.y = msg->twist[i].linear.y;
            agent_vel.z = 0;
            clusters_agent.velocities.push_back(agent_vel);
    }

    clusters_pub.publish(clusters_agent);
}
//
int main(int argc, char **argv)
{

    ros::init(argc, argv, "cadrl_pedsim_node");
    ros::NodeHandle nh;
    ros::Subscriber agents_states_sub = nh.subscribe("/pedsim/agents_states", 0, agentsStatesCallback);
    clusters_pub = nh.advertise<ford_msgs::Clusters>("/upei/cluster/output/clusters",0);
    //
    ros::Rate rate(50);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    //ros::spin();
    return 0;
}

