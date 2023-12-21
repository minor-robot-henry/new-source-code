#include "ros/ros.h"
#include <conio.h>
#include "std_msgs/String.h"

ros::Publisher userinput_pub;
std_msgs::String inputMsg;

int main(int argc,char **argv) {
    ros::init(argc, argv, "UserInput");
    ros::NodeHandle nh;
    
    userinput_pub = nh.advertise<std_msgs::String>("/userinput", 10);
    ROS_INFO("Main in userinputclass");
    inputMsg.data = "";
    
    while (ros::ok()) {
        char userInput = getch();

        if(userInput == 's') {
            inputMsg.data = "Follow";
            ROS_INFO("Current state is: Follow");
        } else if (userInput == 'r') {
            inputMsg.data = "Reset";
            ROS_INFO("Current state is: Reset");
        } else if (userInput == ' ') {
            inputMsg.data = "Idle";
            ROS_INFO("Current state is: Idle");
        } 

        if (inputMsg.data != "") {
            userinput_pub.publish(inputMsg);
        }

        ros::spinOnce();
    }
    return 0;
}