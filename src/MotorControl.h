//****************************************************************************
// TITLE        motorcontroller
// DESCRIPTION	publisher for the drive msgs to send instructions to the rosaria package
// FILE			MotorControl.h
// AUTHOR		R. Schonewille, G. Lutz
// DATE			15--2022
// ***************************************************************************

#pragma once
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "math.h"

// typical values: TURN_SPEED > 0.4, DRIVE_SPEED > 0.5
static float TURN_SPEED = 0.3;
static float DRIVE_SPEED = 0.3;
static float SPEED_LIMIT = 1;
static int MINIMAL_DISTANCE = 1000;
const float ALPHA = 0.75;
static float Z_SPEED = 0.0006;
static float X_SPEED = 0.001;

static float PREV_ANGULAR_VEL = 0;
static float PREV_ANGULAR_VEL_WITHOUT = 0;

class Motorcontrol
{
private:
    ros::Publisher motor_pub;
    geometry_msgs::Twist driveMsg;

public:
    Motorcontrol(ros::NodeHandle *nh)
    {
        motor_pub = nh->advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 10);
    }

    void driveForward()
    {
        driveMsg.linear.x = DRIVE_SPEED;
        driveMsg.angular.z = 0;
        motor_pub.publish(driveMsg);
    }

    void driveBackwards()
    {
        driveMsg.linear.x = -0.2;
        motor_pub.publish(driveMsg);
    }

    void turnRight()
    {
        driveMsg.angular.z = -TURN_SPEED;
        motor_pub.publish(driveMsg);
    }
    
     void turnLeft()
    {
        driveMsg.angular.z = TURN_SPEED;
        motor_pub.publish(driveMsg);
    }

    void resetDrive()
    {
        driveMsg.linear.x = 0;
        driveMsg.angular.z = 0;
        motor_pub.publish(driveMsg);
    }

    void strafeLeft() 
    {
        driveMsg.linear.x = DRIVE_SPEED;
        driveMsg.angular.z = TURN_SPEED;
        motor_pub.publish(driveMsg);
    }

    void strafeRight()
    {
        driveMsg.linear.x = DRIVE_SPEED;
        driveMsg.angular.z = -TURN_SPEED;
        motor_pub.publish(driveMsg);
    }

    void drive(float x, float z) 
    {

        float angleToTarget = atan2(x, z);
        const float DEAD_ZONE = 20.0 * M_PI / 180.0;
        // if(fabs(angleToTarget) < DEAD_ZONE)
        // {
        //     angleToTarget = 0;
        // }        

        float angularVelocity = -1.0 * angleToTarget;

        // angularVelocity = fmax(fmin(angularVelocity, TURN_SPEED), -TURN_SPEED);
        
        // float angularVelocity = -angleToTarget;
        // angularVelocity *= TURN_SPEED;

        // if (x > SPEED_LIMIT) 
        // {
        //     x = 0.5;
        // }

        float linearVelocity = (z - MINIMAL_DISTANCE) * Z_SPEED;

        if ((linearVelocity) > SPEED_LIMIT) 
        {
            linearVelocity = SPEED_LIMIT;
        }

        //float dynamicFactor = 1.0 + ALPHA * z * fabs(angleToTarget);
        //float angularVelocity = -angleToTarget * TURN_SPEED * dynamicFactor;

        driveMsg.linear.x = linearVelocity;
        driveMsg.angular.z = angularVelocity;
        motor_pub.publish(driveMsg);
        //return true
    }
};
