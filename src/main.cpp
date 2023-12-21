//****************************************************************************
// TITLE        Main System ROS wrapper
// DESCRIPTION	implementation of main program to gather sensor info trough their classes and make the data ROS ready
// FILE			main.cpp
// AUTHOR		R. Schonewille
// DATE			25-nov-2022
// ***************************************************************************

#include "ros/ros.h"
#include "visionClass.h"
#include "usClass.h"
#include "MotorControl.h"
#include "std_msgs/String.h"
#include <vector>
#include <conio.h>
#include <array>
#include <thread>

// Declare variables
int operatorID;
float prevOperatorX = 0;
float prevOperatorZ = 0;

usStruct usData;
personCoordinates xyzCoordinates;
personCoordinates absoluteXyzCoordinates;

std::vector<std::array<float, 2>> loggedRoute;

enum caseStates {Init, Idle, Follow, Obstacle};
caseStates currentState = Init;

ros::Subscriber userinput_sub;

// Declare functions
bool isErrorUSDetected(struct usStruct usFunctionData);
bool isUSObjectDetected(struct usStruct usFuctionData);
void addCoordinatesToList(float operatorX, float operatorZ);

/* callbackUserInput():
 * Check the received input in the message from the userInputClass node
 * Set the currentState of the robot on received state
 * 
 * Input: std_msgs::String message
 * Output: void
*/
void callbackUserInput(const std_msgs::String& msg) {
    if (msg.data == "Follow") {
        currentState = Follow;
    } else if (msg.data == "Idle") {
        currentState = Idle;
    } else if (msg.data == "Reset") {
        // Clear the loggedRoute Vector list so that the robot starts from 0
        loggedRoute.clear();
    }
}

int main(int argc,char **argv) {
    ros::init(argc, argv, "StateMachine");
    ros::NodeHandle nodeHandle;

    Vision personCoordinates =  Vision(&nodeHandle);
    us us100s = us(&nodeHandle);
    loggedRoute.clear();
    Motorcontrol pioneer = Motorcontrol(&nodeHandle); //Robot motor

    // Subscribe to the userInput Node and use the callbackUserInput function
    userinput_sub = nodeHandle.subscribe("/userinput", 10, callbackUserInput); 

    while (ros::ok()) {
        // Obstacle detected
        usData = us100s.getSensorValue();
        // Persoon met ID 1 volgen
        operatorID = personCoordinates.getPersonID();
        xyzCoordinates = personCoordinates.getXYZCoordinates();
        absoluteXyzCoordinates = personCoordinates.getAbsoluteXYZCoordinates();
        

        if (isUSObjectDetected(usData) && !isErrorUSDetected(usData)) {
            // TODO: resetDrive() should not be called if an object is 
            //  detected when the current route is being logged
            pioneer.resetDrive();
            ROS_INFO("Drive are set to 0, object detected within 30cm range");
        } else {
            switch(currentState) {
                // Case 1: Init
                case Init:
                    currentState = Idle;
                break; 

                // Case 2: Idle mode
                // Stop the robot and remember the coordinates from the operator who is still moving
                case Idle:
                    // ROS_INFO("Robot is: Idle");
                    ROS_INFO("i: %li", loggedRoute.size());
                    pioneer.resetDrive();
                    addCoordinatesToList(absoluteXyzCoordinates.x, absoluteXyzCoordinates.z);
                break;

                // Case 3: Follow mode
                // Let the robot follow the operator coordinates
                case Follow:
                    // ROS_INFO("Robot is: Follow(ing)");
                    if (operatorID == 1) {
                        // If there is a logged route the robot should follow this route before continuing following the operator:
                        if (!loggedRoute.empty()) {
                            // Read the first element from te route list and send it to the robot motor:
                            std::array<float, 2> coordinates = loggedRoute.front();
                            pioneer.drive(coordinates[0], coordinates[1]);
                            // Remove that coordinate item from the list:
                            loggedRoute.erase(loggedRoute.begin());

                            //FIXME: dit is slechte uitwerking
                            // Make the robot wait a bit so that the coordinates aren't send to the robots motor at once:
                            // std::this_thread::sleep_for(25ms);

                            // Log the size of the logged route (should be size -1 item):
                            ROS_INFO("f: %li", loggedRoute.size());
                        }
                        // Send the current coordinates of the operator to the drive function of the motor:
                        pioneer.drive(xyzCoordinates.x, xyzCoordinates.z);
                    }
                break;
            }
        }
        ros::spinOnce();
    }
    return 0;
}


// Add coordinates to a list if robot is idle to remember the route of the operator
void addCoordinatesToList(float operatorX, float operatorZ) {
    if (loggedRoute.size() < 1000) {
        float differenceX = abs(operatorX - prevOperatorX);
        float differenceZ = abs(operatorZ - prevOperatorZ);
        // ROS_INFO("%f  -  %f", operatorX, prevOperatorX);
        if(differenceX > 1.0 || differenceZ > 1.0) {
            std::array<float, 2> coordinates{{operatorX, operatorZ}};
            loggedRoute.push_back(coordinates);
        }
    }
    prevOperatorX = operatorX;
    prevOperatorZ = operatorZ;
}

// Ultra-soon
bool isUSObjectDetected(usStruct usFunctionData)
{
    if (usData.left || usData.leftCorner || usData.leftFront || usData.rightFront || usData.rightCorner || usData.right || usData.rear)
    {
        return true; 
    }
    return false;
}

bool isErrorUSDetected(usStruct usFunctionData) {
    return usData.error;
}
