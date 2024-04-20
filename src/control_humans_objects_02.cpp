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
#include <std_srvs/Empty.h>

//
using namespace hrvo;
using namespace std;

const float HRVO_TWO_PI = 6.283185307179586f;
static string fixed_frame = "laser";
//
bool start_publishcmd_=true;
ros::ServiceServer srv_start_publishcmd_;
//
ros::Publisher robot_body_pub, robot_velocity_vector_pub, ref_velocity_pub;
ros::Publisher cmd_h28, cmd_h29, cmd_h30, cmd_h31, cmd_h32, cmd_h33, cmd_h34, cmd_h35, cmd_h36, cmd_h37, cmd_h38, cmd_h39;
ros::Publisher cmd_h40, cmd_h41, cmd_h42, cmd_h43, cmd_h44, cmd_h45, cmd_h46, cmd_h47, cmd_h48, cmd_h49;
ros::Publisher cmd_h50, cmd_h51, cmd_h52, cmd_h53, cmd_h54, cmd_h55, cmd_h56, cmd_h57, cmd_h58, cmd_h59;
ros::Publisher cmd_h60, cmd_h61, cmd_h62, cmd_h63, cmd_h64, cmd_h65, cmd_h66, cmd_h67, cmd_h68, cmd_h69;
ros::Publisher cmd_h70, cmd_h71, cmd_h72, cmd_h73, cmd_h74, cmd_h75, cmd_h76, cmd_h77, cmd_h78, cmd_h79;
//
int goal_selection[80]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
Vector2 globalgoal;
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
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
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
    // Convert to -pi-pi;
    theta_cur = angles::normalize_angle(theta_cur);
    double diff = angles::shortest_angular_distance(theta_cur, theta_new);
    //double diff = theta_new - theta_cur;
    double omega = 0;
    omega = diff/(1.0*1);//1.8*1
    cmd_vel.angular.z = omega;
    return cmd_vel;
}
//
void agentsStatesCallback(const gazebo_msgs::ModelStatesConstPtr & msg){
if(start_publishcmd_){
    Simulator simulator;
    visualization_msgs::Marker robotmarker, velocitymarker;
    visualization_msgs::MarkerArray robotmarkerarray, velocitymarkerarray;
    geometry_msgs::Twist ref_velocity;
    gazebo_msgs::ModelStates ref_velocity_array;

    simulator.setTimeStep(0.25f); //0.25
    for (std::size_t i = 12; i < msg->name.size(); ++i) {
        float posx = msg->pose[i].position.x;   float posy = msg->pose[i].position.y;
        float goal1x = msg->twist[i].angular.x; float goal1y = msg->twist[i].angular.y;
        float goal2x = msg->twist[i].linear.y;  float goal2y = msg->twist[i].linear.z;
        float distogoal1 = sqrt((posx-goal1x)*(posx-goal1x)+(posy-goal1y)*(posy-goal1y));
        float distogoal2 = sqrt((posx-goal2x)*(posx-goal2x)+(posy-goal2y)*(posy-goal2y));
        Vector2 tmpgoal;
        if(distogoal2 < 0.5){goal_selection[i]=0;  }
        if(distogoal1 < 0.5){goal_selection[i]=1;  }
        if (goal_selection[i]==0){
            tmpgoal = Vector2(goal1x, goal1y);
        }else{
            tmpgoal = Vector2(goal2x, goal2y);
        }
        const Vector2 goal =tmpgoal;
        //ROS_INFO("Goals x = %f, y= %f", goal.getX(), goal.getY());
        const Vector2 position = Vector2(posx, posy);
        float theta_cur = angles::normalize_angle(tf::getYaw(msg->pose[i].orientation));
        float velx = cos(theta_cur) * msg->twist[i].linear.x;
        float vely = sin(theta_cur) * msg->twist[i].linear.x;
        const Vector2 vel = Vector2(velx, vely);
        //ROS_INFO("Goal Position: %f, %f", msg->twist[i].angular.x, msg->twist[i].angular.y);
        //simulator.addAgent(position, simulator.addGoal(goal));
        simulator.addAgent(position,                // const Vector2 &position
                           simulator.addGoal(goal), // std::size_t goalNo
                           10.0f,                   // float neighborDist 8
                           16.0f,                      // std::size_t maxNeighbors 15
                           0.6f,                    // float radius 1.0
                           1.0f,                    // float goalRadius
                           0.4f,                    // float prefSpeed
                           0.4f,                    // float maxSpeed
                           0.0f,                    // float uncertaintyOffset
                           1.5f,                    // float maxAccel
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
        velocitymarker = createVector(pos.getX(),pos.getY(),1.2,2*vel.getX(),2*vel.getY(),i);
        velocitymarkerarray.markers.push_back(velocitymarker);
        ref_velocity.linear.x = vel.getX();
        ref_velocity.linear.y = vel.getY();
        ref_velocity_array.twist.push_back(ref_velocity);
    }
    //
    geometry_msgs::Twist vel_h28, vel_h29, vel_h30, vel_h31, vel_h32, vel_h33, vel_h34, vel_h35, vel_h36, vel_h37, vel_h38, vel_h39;
    geometry_msgs::Twist vel_h40, vel_h41, vel_h42, vel_h43, vel_h44, vel_h45, vel_h46, vel_h47, vel_h48, vel_h49;
    geometry_msgs::Twist vel_h50, vel_h51, vel_h52, vel_h53, vel_h54, vel_h55, vel_h56, vel_h57, vel_h58, vel_h59;
    geometry_msgs::Twist vel_h60, vel_h61, vel_h62, vel_h63, vel_h64, vel_h65, vel_h66, vel_h67, vel_h68, vel_h69;
    geometry_msgs::Twist vel_h70, vel_h71, vel_h72, vel_h73, vel_h74, vel_h75, vel_h76, vel_h77, vel_h78, vel_h79;
    //
    geometry_msgs::Pose pose28 = msg->pose[28]; Vector2 vel28 = simulator.getAgentVelocity(28-12);
    vel_h28 = computeControlCommand(vel28, pose28); cmd_h28.publish(vel_h28);
    geometry_msgs::Pose pose29 = msg->pose[29]; Vector2 vel29 = simulator.getAgentVelocity(29-12);
    vel_h29 = computeControlCommand(vel29, pose29); cmd_h29.publish(vel_h29);
    //
    geometry_msgs::Pose pose30 = msg->pose[30]; Vector2 vel30 = simulator.getAgentVelocity(30-12);
    vel_h30 = computeControlCommand(vel30, pose30); cmd_h30.publish(vel_h30);
    geometry_msgs::Pose pose31 = msg->pose[31]; Vector2 vel31 = simulator.getAgentVelocity(31-12);
    vel_h31 = computeControlCommand(vel31, pose31); cmd_h31.publish(vel_h31);
    geometry_msgs::Pose pose32 = msg->pose[32]; Vector2 vel32 = simulator.getAgentVelocity(32-12);
    vel_h32 = computeControlCommand(vel32, pose32); cmd_h32.publish(vel_h32);
    geometry_msgs::Pose pose33 = msg->pose[33]; Vector2 vel33 = simulator.getAgentVelocity(33-12);
    vel_h33 = computeControlCommand(vel33, pose33); cmd_h33.publish(vel_h33);
    geometry_msgs::Pose pose34 = msg->pose[34]; Vector2 vel34 = simulator.getAgentVelocity(34-12);
    vel_h34 = computeControlCommand(vel34, pose34); cmd_h34.publish(vel_h34);
    geometry_msgs::Pose pose35 = msg->pose[35]; Vector2 vel35 = simulator.getAgentVelocity(35-12);
    vel_h35 = computeControlCommand(vel35, pose35); cmd_h35.publish(vel_h35);
    geometry_msgs::Pose pose36 = msg->pose[36]; Vector2 vel36 = simulator.getAgentVelocity(36-12);
    vel_h36 = computeControlCommand(vel36, pose36); cmd_h36.publish(vel_h36);
    geometry_msgs::Pose pose37 = msg->pose[37]; Vector2 vel37 = simulator.getAgentVelocity(37-12);
    vel_h37 = computeControlCommand(vel37, pose37); cmd_h37.publish(vel_h37);
    geometry_msgs::Pose pose38 = msg->pose[38]; Vector2 vel38 = simulator.getAgentVelocity(38-12);
    vel_h38 = computeControlCommand(vel38, pose38); cmd_h38.publish(vel_h38);
    geometry_msgs::Pose pose39 = msg->pose[39]; Vector2 vel39 = simulator.getAgentVelocity(39-12);
    vel_h39 = computeControlCommand(vel39, pose39); cmd_h39.publish(vel_h39);

    geometry_msgs::Pose pose40 = msg->pose[40]; Vector2 vel40 = simulator.getAgentVelocity(40-12);
    vel_h40 = computeControlCommand(vel40, pose40); cmd_h40.publish(vel_h40);
    geometry_msgs::Pose pose41 = msg->pose[41]; Vector2 vel41 = simulator.getAgentVelocity(41-12);
    vel_h41 = computeControlCommand(vel41, pose41); cmd_h41.publish(vel_h41);
    geometry_msgs::Pose pose42 = msg->pose[42]; Vector2 vel42 = simulator.getAgentVelocity(42-12);
    vel_h42 = computeControlCommand(vel42, pose42); cmd_h42.publish(vel_h42);
    geometry_msgs::Pose pose43 = msg->pose[43]; Vector2 vel43 = simulator.getAgentVelocity(43-12);
    vel_h43 = computeControlCommand(vel43, pose43); cmd_h43.publish(vel_h43);
    geometry_msgs::Pose pose44 = msg->pose[44]; Vector2 vel44 = simulator.getAgentVelocity(44-12);
    vel_h44 = computeControlCommand(vel44, pose44); cmd_h44.publish(vel_h44);
    geometry_msgs::Pose pose45 = msg->pose[45]; Vector2 vel45 = simulator.getAgentVelocity(45-12);
    vel_h45 = computeControlCommand(vel45, pose45); cmd_h45.publish(vel_h45);
    geometry_msgs::Pose pose46 = msg->pose[46]; Vector2 vel46 = simulator.getAgentVelocity(46-12);
    vel_h46 = computeControlCommand(vel46, pose46); cmd_h46.publish(vel_h46);
    geometry_msgs::Pose pose47 = msg->pose[47]; Vector2 vel47 = simulator.getAgentVelocity(47-12);
    vel_h47 = computeControlCommand(vel47, pose47); cmd_h47.publish(vel_h47);
    geometry_msgs::Pose pose48 = msg->pose[48]; Vector2 vel48 = simulator.getAgentVelocity(48-12);
    vel_h48 = computeControlCommand(vel48, pose48); cmd_h48.publish(vel_h48);
    geometry_msgs::Pose pose49 = msg->pose[49]; Vector2 vel49 = simulator.getAgentVelocity(49-12);
    vel_h49 = computeControlCommand(vel49, pose49); cmd_h49.publish(vel_h49);

    geometry_msgs::Pose pose50 = msg->pose[50]; Vector2 vel50 = simulator.getAgentVelocity(50-12);
    vel_h50 = computeControlCommand(vel50, pose50); cmd_h50.publish(vel_h50);
    geometry_msgs::Pose pose51 = msg->pose[51]; Vector2 vel51 = simulator.getAgentVelocity(51-12);
    vel_h51 = computeControlCommand(vel51, pose51); cmd_h51.publish(vel_h51);
    geometry_msgs::Pose pose52 = msg->pose[52]; Vector2 vel52 = simulator.getAgentVelocity(52-12);
    vel_h52 = computeControlCommand(vel52, pose52); cmd_h52.publish(vel_h52);
    geometry_msgs::Pose pose53 = msg->pose[53]; Vector2 vel53 = simulator.getAgentVelocity(53-12);
    vel_h53 = computeControlCommand(vel53, pose53); cmd_h53.publish(vel_h53);
    geometry_msgs::Pose pose54 = msg->pose[54]; Vector2 vel54 = simulator.getAgentVelocity(54-12);
    vel_h54 = computeControlCommand(vel54, pose54); cmd_h54.publish(vel_h54);
    geometry_msgs::Pose pose55 = msg->pose[55]; Vector2 vel55 = simulator.getAgentVelocity(55-12);
    vel_h55 = computeControlCommand(vel55, pose55); cmd_h55.publish(vel_h55);
    geometry_msgs::Pose pose56 = msg->pose[56]; Vector2 vel56 = simulator.getAgentVelocity(56-12);
    vel_h56 = computeControlCommand(vel56, pose56); cmd_h56.publish(vel_h56);
    geometry_msgs::Pose pose57 = msg->pose[57]; Vector2 vel57 = simulator.getAgentVelocity(57-12);
    vel_h57 = computeControlCommand(vel57, pose57); cmd_h57.publish(vel_h57);
    geometry_msgs::Pose pose58 = msg->pose[58]; Vector2 vel58 = simulator.getAgentVelocity(58-12);
    vel_h58 = computeControlCommand(vel58, pose58); cmd_h58.publish(vel_h58);
    geometry_msgs::Pose pose59 = msg->pose[59]; Vector2 vel59 = simulator.getAgentVelocity(59-12);
    vel_h59 = computeControlCommand(vel59, pose59); cmd_h59.publish(vel_h59);

    geometry_msgs::Pose pose60 = msg->pose[60]; Vector2 vel60 = simulator.getAgentVelocity(60-12);
    vel_h60 = computeControlCommand(vel60, pose60); cmd_h60.publish(vel_h60);
    geometry_msgs::Pose pose61 = msg->pose[61]; Vector2 vel61 = simulator.getAgentVelocity(61-12);
    vel_h61 = computeControlCommand(vel61, pose61); cmd_h61.publish(vel_h61);
    geometry_msgs::Pose pose62 = msg->pose[62]; Vector2 vel62 = simulator.getAgentVelocity(62-12);
    vel_h62 = computeControlCommand(vel62, pose62); cmd_h62.publish(vel_h62);
    geometry_msgs::Pose pose63 = msg->pose[63]; Vector2 vel63 = simulator.getAgentVelocity(63-12);
    vel_h63 = computeControlCommand(vel63, pose63); cmd_h63.publish(vel_h63);
    geometry_msgs::Pose pose64 = msg->pose[64]; Vector2 vel64 = simulator.getAgentVelocity(64-12);
    vel_h64 = computeControlCommand(vel64, pose64); cmd_h64.publish(vel_h64);
    geometry_msgs::Pose pose65 = msg->pose[65]; Vector2 vel65 = simulator.getAgentVelocity(65-12);
    vel_h65 = computeControlCommand(vel65, pose65); cmd_h65.publish(vel_h65);
    geometry_msgs::Pose pose66 = msg->pose[66]; Vector2 vel66 = simulator.getAgentVelocity(66-12);
    vel_h66 = computeControlCommand(vel66, pose66); cmd_h66.publish(vel_h66);
    geometry_msgs::Pose pose67 = msg->pose[67]; Vector2 vel67 = simulator.getAgentVelocity(67-12);
    vel_h67 = computeControlCommand(vel67, pose67); cmd_h67.publish(vel_h67);
    geometry_msgs::Pose pose68 = msg->pose[68]; Vector2 vel68 = simulator.getAgentVelocity(68-12);
    vel_h68 = computeControlCommand(vel68, pose68); cmd_h68.publish(vel_h68);
    geometry_msgs::Pose pose69 = msg->pose[69]; Vector2 vel69 = simulator.getAgentVelocity(69-12);
    vel_h69 = computeControlCommand(vel69, pose69); cmd_h69.publish(vel_h69);

    geometry_msgs::Pose pose70 = msg->pose[70]; Vector2 vel70 = simulator.getAgentVelocity(70-12);
    vel_h70 = computeControlCommand(vel70, pose70); cmd_h70.publish(vel_h70);
    geometry_msgs::Pose pose71 = msg->pose[71]; Vector2 vel71 = simulator.getAgentVelocity(71-12);
    vel_h71 = computeControlCommand(vel71, pose71); cmd_h71.publish(vel_h71);
    geometry_msgs::Pose pose72 = msg->pose[72]; Vector2 vel72 = simulator.getAgentVelocity(72-12);
    vel_h72 = computeControlCommand(vel72, pose72); cmd_h72.publish(vel_h72);
    geometry_msgs::Pose pose73 = msg->pose[73]; Vector2 vel73 = simulator.getAgentVelocity(73-12);
    vel_h73 = computeControlCommand(vel73, pose73); cmd_h73.publish(vel_h73);
    geometry_msgs::Pose pose74 = msg->pose[74]; Vector2 vel74 = simulator.getAgentVelocity(74-12);
    vel_h74 = computeControlCommand(vel74, pose74); cmd_h74.publish(vel_h74);
    geometry_msgs::Pose pose75 = msg->pose[75]; Vector2 vel75 = simulator.getAgentVelocity(75-12);
    vel_h75 = computeControlCommand(vel75, pose75); cmd_h75.publish(vel_h75);
    geometry_msgs::Pose pose76 = msg->pose[76]; Vector2 vel76 = simulator.getAgentVelocity(76-12);
    vel_h76 = computeControlCommand(vel76, pose76); cmd_h76.publish(vel_h76);
    geometry_msgs::Pose pose77 = msg->pose[77]; Vector2 vel77 = simulator.getAgentVelocity(77-12);
    vel_h77 = computeControlCommand(vel77, pose77); cmd_h77.publish(vel_h77);
    geometry_msgs::Pose pose78 = msg->pose[78]; Vector2 vel78 = simulator.getAgentVelocity(78-12);
    vel_h78 = computeControlCommand(vel78, pose78); cmd_h78.publish(vel_h78);
    geometry_msgs::Pose pose79 = msg->pose[79]; Vector2 vel79 = simulator.getAgentVelocity(79-12);
    vel_h79 = computeControlCommand(vel79, pose79); cmd_h79.publish(vel_h79);

    robot_body_pub.publish(robotmarkerarray);
    robot_velocity_vector_pub.publish(velocitymarkerarray);
    ref_velocity_pub.publish(ref_velocity_array);
    /*
    double ah1 = -2.3101;
    double ah2 =  0.8394;
    double diffah = angles::shortest_angular_distance(ah1, ah2);
    double ah3 = -2.3143;
    double ah4 =  0.8084;
    double diffah1 = angles::shortest_angular_distance(ah3, ah4);
    ROS_INFO("Diff_AH = %f and %f", diffah, diffah1);
    */
}
}
//
bool onStartPublishCmd(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    start_publishcmd_ = !start_publishcmd_;
    //stop_human_data = false;
    ROS_INFO("Start and Stop publish data = %d", start_publishcmd_);

   return true;
}

//
int main(int argc, char **argv)
{

    ros::init(argc, argv, "control_humans_objects_using_hrvo");
    ros::NodeHandle nh;
    ros::Subscriber agents_states_sub;
    //
    srv_start_publishcmd_ = nh.advertiseService("/multirobot/start_publishcmd", &onStartPublishCmd);

    // vis
    robot_body_pub = nh.advertise<visualization_msgs::MarkerArray>("hrvo/robot_body", 0);
    robot_velocity_vector_pub = nh.advertise<visualization_msgs::MarkerArray>("hrvo/robot_velocity_vector",0);
    // data
    ref_velocity_pub = nh.advertise<gazebo_msgs::ModelStates>("/hrvo/ref_velocity",1);
    //
    cmd_h28 = nh.advertise<geometry_msgs::Twist>("robot_28/cmd_vel",1);
    cmd_h29 = nh.advertise<geometry_msgs::Twist>("robot_29/cmd_vel",1);
    cmd_h30 = nh.advertise<geometry_msgs::Twist>("robot_30/cmd_vel",1);
    cmd_h31 = nh.advertise<geometry_msgs::Twist>("robot_31/cmd_vel",1);
    cmd_h32 = nh.advertise<geometry_msgs::Twist>("robot_32/cmd_vel",1);
    cmd_h33 = nh.advertise<geometry_msgs::Twist>("robot_33/cmd_vel",1);
    cmd_h34 = nh.advertise<geometry_msgs::Twist>("robot_34/cmd_vel",1);
    cmd_h35 = nh.advertise<geometry_msgs::Twist>("robot_35/cmd_vel",1);
    cmd_h36 = nh.advertise<geometry_msgs::Twist>("robot_36/cmd_vel",1);
    cmd_h37 = nh.advertise<geometry_msgs::Twist>("robot_37/cmd_vel",1);
    cmd_h38 = nh.advertise<geometry_msgs::Twist>("robot_38/cmd_vel",1);
    cmd_h39 = nh.advertise<geometry_msgs::Twist>("robot_39/cmd_vel",1);

    cmd_h40 = nh.advertise<geometry_msgs::Twist>("robot_40/cmd_vel",1);
    cmd_h41 = nh.advertise<geometry_msgs::Twist>("robot_41/cmd_vel",1);
    cmd_h42 = nh.advertise<geometry_msgs::Twist>("robot_42/cmd_vel",1);
    cmd_h43 = nh.advertise<geometry_msgs::Twist>("robot_43/cmd_vel",1);
    cmd_h44 = nh.advertise<geometry_msgs::Twist>("robot_44/cmd_vel",1);
    cmd_h45 = nh.advertise<geometry_msgs::Twist>("robot_45/cmd_vel",1);
    cmd_h46 = nh.advertise<geometry_msgs::Twist>("robot_46/cmd_vel",1);
    cmd_h47 = nh.advertise<geometry_msgs::Twist>("robot_47/cmd_vel",1);
    cmd_h48 = nh.advertise<geometry_msgs::Twist>("robot_48/cmd_vel",1);
    cmd_h49 = nh.advertise<geometry_msgs::Twist>("robot_49/cmd_vel",1);

    cmd_h50 = nh.advertise<geometry_msgs::Twist>("robot_50/cmd_vel",1);
    cmd_h51 = nh.advertise<geometry_msgs::Twist>("robot_51/cmd_vel",1);
    cmd_h52 = nh.advertise<geometry_msgs::Twist>("robot_52/cmd_vel",1);
    cmd_h53 = nh.advertise<geometry_msgs::Twist>("robot_53/cmd_vel",1);
    cmd_h54 = nh.advertise<geometry_msgs::Twist>("robot_54/cmd_vel",1);
    cmd_h55 = nh.advertise<geometry_msgs::Twist>("robot_55/cmd_vel",1);
    cmd_h56 = nh.advertise<geometry_msgs::Twist>("robot_56/cmd_vel",1);
    cmd_h57 = nh.advertise<geometry_msgs::Twist>("robot_57/cmd_vel",1);
    cmd_h58 = nh.advertise<geometry_msgs::Twist>("robot_58/cmd_vel",1);
    cmd_h59 = nh.advertise<geometry_msgs::Twist>("robot_59/cmd_vel",1);

    cmd_h60 = nh.advertise<geometry_msgs::Twist>("robot_60/cmd_vel",1);
    cmd_h61 = nh.advertise<geometry_msgs::Twist>("robot_61/cmd_vel",1);
    cmd_h62 = nh.advertise<geometry_msgs::Twist>("robot_62/cmd_vel",1);
    cmd_h63 = nh.advertise<geometry_msgs::Twist>("robot_63/cmd_vel",1);
    cmd_h64 = nh.advertise<geometry_msgs::Twist>("robot_64/cmd_vel",1);
    cmd_h65 = nh.advertise<geometry_msgs::Twist>("robot_65/cmd_vel",1);
    cmd_h66 = nh.advertise<geometry_msgs::Twist>("robot_66/cmd_vel",1);
    cmd_h67 = nh.advertise<geometry_msgs::Twist>("robot_67/cmd_vel",1);
    cmd_h68 = nh.advertise<geometry_msgs::Twist>("robot_68/cmd_vel",1);
    cmd_h69 = nh.advertise<geometry_msgs::Twist>("robot_69/cmd_vel",1);

    cmd_h70 = nh.advertise<geometry_msgs::Twist>("robot_70/cmd_vel",1);
    cmd_h71 = nh.advertise<geometry_msgs::Twist>("robot_71/cmd_vel",1);
    cmd_h72 = nh.advertise<geometry_msgs::Twist>("robot_72/cmd_vel",1);
    cmd_h73 = nh.advertise<geometry_msgs::Twist>("robot_73/cmd_vel",1);
    cmd_h74 = nh.advertise<geometry_msgs::Twist>("robot_74/cmd_vel",1);
    cmd_h75 = nh.advertise<geometry_msgs::Twist>("robot_75/cmd_vel",1);
    cmd_h76 = nh.advertise<geometry_msgs::Twist>("robot_76/cmd_vel",1);
    cmd_h77 = nh.advertise<geometry_msgs::Twist>("robot_77/cmd_vel",1);
    cmd_h78 = nh.advertise<geometry_msgs::Twist>("robot_78/cmd_vel",1);
    cmd_h79 = nh.advertise<geometry_msgs::Twist>("robot_79/cmd_vel",1);
    //
    agents_states_sub = nh.subscribe("/stage/agents_states", 5, &agentsStatesCallback);
    //

    ros::Rate rate(10);
    while (ros::ok())
    {


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

