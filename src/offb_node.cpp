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

mavros_msgs::State current_state;
nav_msgs::Odometry current_odometry;

// MQTT message storage and access
json data;
mqtt::async_client *client = NULL;
ros::Timer timer;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void odometry_cb(const nav_msgs::Odometry::ConstPtr& msg){
    current_odometry = *msg;
}

/////////////////////////////////////////////////////////////////////////////
//TODO: Create a callback for MQTT using the callback class
/**
 * Local callback & listener class for use with the client connection.
 * This is primarily intended to receive messages, but it will also monitor
 * the connection to the broker. If the connection is lost, it will attempt
 * to restore the connection and re-subscribe to the topic.
 */
class callback : public virtual mqtt::callback,
					public virtual mqtt::iaction_listener

{
	// Counter for the number of connection retries
	int nretry_;
	// The MQTT client
	mqtt::async_client& cli_;
	// Options to use if we need to reconnect
	mqtt::connect_options& connOpts_;
	// An action listener to display the result of actions.
	action_listener subListener_;

	// This deomonstrates manually reconnecting to the broker by calling
	// connect() again. This is a possibility for an application that keeps
	// a copy of it's original connect_options, or if the app wants to
	// reconnect with different options.
	// Another way this can be done manually, if using the same options, is
	// to just call the async_client::reconnect() method.
	void reconnect() {
		std::this_thread::sleep_for(std::chrono::milliseconds(2500));
		try {
			cli_.connect(connOpts_, nullptr, *this);
		}
		catch (const mqtt::exception& exc) {
			std::cerr << "Error: " << exc.what() << std::endl;
			exit(1);
		}
	}

	// Re-connection failure
	void on_failure(const mqtt::token& tok) override {
		std::cout << "Connection attempt failed" << std::endl;
		if (++nretry_ > N_RETRY_ATTEMPTS)
			exit(1);
		reconnect();
	}

	// (Re)connection success
	// Either this or connected() can be used for callbacks.
	void on_success(const mqtt::token& tok) override {}

	// (Re)connection success
	void connected(const std::string& cause) override {
		std::cout << "\nConnection success" << std::endl;
		std::cout << "\nSubscribing to topic '" << TOPIC << "'\n"
			<< "\tfor client " << CLIENT_ID
			<< " using QoS" << QOS << "\n"
			<< "\nPress Q<Enter> to quit\n" << std::endl;

		cli_.subscribe(TOPIC, QOS, nullptr, subListener_);
	}

	// Callback for when the connection is lost.
	// This will initiate the attempt to manually reconnect.
	void connection_lost(const std::string& cause) override {
		std::cout << "\nConnection lost" << std::endl;
		if (!cause.empty())
			std::cout << "\tcause: " << cause << std::endl;

		std::cout << "Reconnecting..." << std::endl;
		nretry_ = 0;
		reconnect();
	}

	// Callback for when a message arrives.
	void message_arrived(mqtt::const_message_ptr msg) override {
		std::cout << "Message arrived" << std::endl;
		std::cout << "\ttopic: '" << msg->get_topic() << "'" << std::endl;
		std::cout << "\tpayload: '" << msg->to_string() << "'\n" << std::endl;
	}

	void delivery_complete(mqtt::delivery_token_ptr token) override {}

public:
	callback(mqtt::async_client& cli, mqtt::connect_options& connOpts)
				: nretry_(0), cli_(cli), connOpts_(connOpts), subListener_("Subscription") {}
};

/////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{

    // Initialize ros node
    ros::init(argc,argv,"MAVROS_Offboard_Control_Node");
    ros::NodeHandle nh;

    // Subscribe to state and odometry
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state",10,state_cb);
    ros::Subscriber odometry_sub = nh.subscribe<nav_msgs::Odometry>("mavros/odometry/in",10, odometry_cb);

    // Publish to setpoint and velocity
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel",10);

    // Create service clients for arming and set mode
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(5.0); //Publishing rate faster than 2Hz for setpoint publising

    // Wait for connection to be established
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("ROS is functioning correctly and is connected to MAVROS");

    // Create MQTT client and connect
    client = new mqtt::async_client("127.0.0.1:1883", "ROS-consumer", 0);

    ROS_INFO("Connecting to MQTT broker...");
    client->connect();
    ROS_INFO("Connected to MQTT broker");

    client->set_callback(&mqtt_cb);
    client->subscribe("dlstreamer-publisher",0);
    client->start_consuming(); 
    ROS_INFO("Subscribed to MQTT topic and ready for consumption");
    //timer = nh.createTimer(ros::Duration(1.0/10.0), mqtt_cb, false, true);

    // Create poses for the offboard mode 
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0; pose.pose.position.y = 0; pose.pose.position.z = 2;

    for(int i = 100; ros::ok() && i>0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Local position setpoint sent");

    // Set offboard mode
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    ROS_INFO("OFFBOARD mode enabled");

    // Set arm command
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ROS_INFO("Vehicle arm command prepared");

    ROS_INFO("Starting simulation loop");
    mqtt::const_message_ptr messagePointer; // Construct a message pointer to hold an incoming message.
    ros::Time last_request = ros::Time::now();

    ROS_INFO("Starting offboard node and running for 5 seconds");
    while(ros::ok()){

        if(current_state.mode!="OFFBOARD"&&(ros::Time::now()-last_request>ros::Duration(5.0))){

            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
                ROS_INFO("Arming vehicle and running arm service for 5 seconds");
            }

        last_request = ros::Time::now();
        local_pos_pub.publish(pose);

        }else{

            if(!current_state.armed && (ros::Time::now()-last_request>ros::Duration(5.0))){

                if(arming_client.call(arm_cmd)&&arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }

            last_request = ros::Time::now();
            local_pos_pub.publish(pose);

            }else{

                /*std::cout<<"X_max: "<<data["objects"][0]["detection"]["bounding_box"]["x_max"];
                std::cout<<" ; X_min: "<<data["objects"][0]["detection"]["bounding_box"]["x_min"];
                std::cout<<" ; Y_max: "<<data["objects"][0]["detection"]["bounding_box"]["y_max"];
                std::cout<<" ; Y_min: "<<data["objects"][0]["detection"]["bounding_box"]["y_min"]<<std::endl;*/

                pose.pose.position.x = std::round(current_odometry.pose.pose.position.x*10)/10; 
                pose.pose.position.y = std::round(current_odometry.pose.pose.position.y*10)/10; 
                pose.pose.position.z = 2;

                if(data["objects"][0]["detection"]["bounding_box"]["y_max"]>0.9){
                    pose.pose.position.x = current_odometry.pose.pose.position.x - 0.1; 

                }else{if(data["objects"][0]["detection"]["bounding_box"]["y_min"]<0.1){  
                    pose.pose.position.x = current_odometry.pose.pose.position.x + 0.1; 

                    }
                }

                if(data["objects"][0]["detection"]["bounding_box"]["x_max"]>0.9){
                    pose.pose.position.x = current_odometry.pose.pose.position.y - 0.1; 

                }else{if(data["objects"][0]["detection"]["bounding_box"]["x_min"]<0.1){  
                    pose.pose.position.x = current_odometry.pose.pose.position.y + 0.1; 

                    }
                }

                local_pos_pub.publish(pose);
            }
        }

        ros::spinOnce();
        rate.sleep();

    }
    return 0;
}