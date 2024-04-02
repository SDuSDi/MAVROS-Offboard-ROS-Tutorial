/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo Classic SITL
 */

// ROS libraries and dependencies
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>

// MQTT library
#include <mqtt/async_client.h>
// JSON library
#include <nlohmann/json.hpp>
using json = nlohmann::json; // for convenience
// Callback file
#include "classes.cpp"

mavros_msgs::State current_state;
nav_msgs::Odometry current_odometry;

// MQTT message storage and access
mqtt::async_client *client = NULL;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void odometry_cb(const nav_msgs::Odometry::ConstPtr& msg){
    current_odometry = *msg;
}

int main(int argc, char **argv)
{

    // Initialize ros node
    ros::init(argc,argv,"MAVROS_Offboard_Control_Node");
    ros::NodeHandle nh;

    // Subscribe to state and odometry
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state",10,state_cb);
    ros::Subscriber odometry_sub = nh.subscribe<nav_msgs::Odometry>("mavros/odometry/in",10, odometry_cb);

    // Publish to cmd velocity
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel",10);

    // Create service clients for arming and set mode
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    float speed = 20.0;
    ros::Rate rate(speed); //Publishing rate faster than 2Hz for setpoint publising

    // Wait for connection to be established
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("ROS is functioning correctly and is connected to MAVROS");

    // Create MQTT client and connect
    client = new mqtt::async_client("127.0.0.1:1883", "ROS-consumer", 0);

    // Set connection options
	mqtt::connect_options connOpts;
	connOpts.set_clean_session(false);

    // Create callback object and set callback for use
    callback mqtt_cb(*client, connOpts);
    client->set_callback(mqtt_cb);

    client->start_consuming();
    ROS_INFO("MQTT client ready for consumption");

    ROS_INFO("Connecting to MQTT broker...");
    client->connect(connOpts);
    ROS_INFO("Connected to MQTT broker");

    while(!client->is_connected()){
        ROS_INFO("Failed to connect to MQTT broker. Retrying...");
        rate.sleep();
    }

    // Set offboard mode
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    ROS_INFO("OFFBOARD mode enabled");

    // Set arm command
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ROS_INFO("Vehicle arm command prepared");

    ROS_INFO("Starting simulation loop");
    ros::Time last_request = ros::Time::now();

    double estimated_x = 0.0;
    double estimated_y = 0.0;

    ROS_INFO("Starting offboard node and running for 5 seconds");
    while(ros::ok()){

        geometry_msgs::TwistStamped vel;

        vel.twist.linear.x=0; vel.twist.linear.y=0; vel.twist.linear.z=0;
        vel.twist.angular.x=0; vel.twist.angular.y=0; vel.twist.angular.z=0;

        if(current_state.mode!="OFFBOARD"&&(ros::Time::now()-last_request>ros::Duration(5.0))){

            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
                ROS_INFO("Arming vehicle and running arm service for 5 seconds");
            }

        last_request = ros::Time::now();

        }else{

            if(!current_state.armed && (ros::Time::now()-last_request>ros::Duration(5.0))){

                if(arming_client.call(arm_cmd)&&arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }

            last_request = ros::Time::now();

            }else{

                if(data["objects"][0]["detection"]["bounding_box"]["y_max"]>0.9){
                    estimated_x += 0.5 * 1.0/speed; 

                }else{if(data["objects"][0]["detection"]["bounding_box"]["y_min"]<0.1){ 
                    estimated_x += -0.5 * 1.0/speed;  

                    }
                }

                if(data["objects"][0]["detection"]["bounding_box"]["x_max"]>0.9){
                    estimated_y += 0.4 * 1.0/speed;   

                }else{if(data["objects"][0]["detection"]["bounding_box"]["x_min"]<0.1){  
                    estimated_y += -0.4 * 1.0/speed;  

                    }
                }

                int kp = 1;
                vel.twist.linear.x = kp * (estimated_x - current_odometry.pose.pose.position.x);
                vel.twist.linear.y = kp * (estimated_y - current_odometry.pose.pose.position.y);
                vel.twist.linear.z = kp * (2 - current_odometry.pose.pose.position.z);

            }
        }

        vel_pub.publish(vel); 

        ros::spinOnce();
        rate.sleep();

    }
    return 0;
}