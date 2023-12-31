//****************************************************************************
// TITLE        visionClass
// DESCRIPTION	Class definition for a vision sensor
// FILE			visionClass.h
// AUTHOR		R. Schonewille
// DATE			25-nov-2022
// ***************************************************************************

#pragma once
#include <ros/ros.h>
#include "sscmain/rfmVision.h"
struct personCoordinates {float x; float y; float z;};

// Vision Class

class Vision
{
private:
    //ros variable
    ros::Subscriber vision_sub;
    int personID = 0;

	// private variables
    struct personCoordinates personCoordinates;
    struct personCoordinates absolutePersonCoordinates;

public:
	// constructors
    // Ros vision subscriber 
    Vision(ros::NodeHandle *nh)
    {  
        vision_sub = nh->subscribe("/VisionData", 1000, &Vision::callback_data, this);
    }

    // Member functions for visionClass
     //Getters
    int getPersonID()
    {
        return personID;
    }

    struct personCoordinates getXYZCoordinates()
    {
        return personCoordinates;
    }

    struct personCoordinates getAbsoluteXYZCoordinates()
    {
        return absolutePersonCoordinates;
    }
    
    //Callback for ROS
    void callback_data(const sscmain::rfmVision& msg)
    {
        this -> personID = msg.front().i;        
        this -> personCoordinates.x = msg.front().x;
        this -> personCoordinates.y = msg.front().y;
        this -> personCoordinates.z = msg.front().z;

        this -> absolutePersonCoordinates.x = msg.back().x;
        this -> absolutePersonCoordinates.y = msg.back().y;
        this -> absolutePersonCoordinates.z = msg.back().z;
    }
};

