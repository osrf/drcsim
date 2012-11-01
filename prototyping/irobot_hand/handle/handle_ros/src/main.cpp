/**
 * \file main.cpp
 *
 * ROS wrapper for HANDLE
 *
 * Written under government funding for ARM-H project.
 *
 * \author Ben Axelrod
 * \date   March 2012
 * \copyright Copyright iRobot Corporation, 2012
 **/

// #include "../../../HandleLib/handleTcp.hpp"
// #include "../../../HandleLib/handleControl.hpp"
#include <handle_lib/handleTcp.h>
#include <handle_lib/handleControl.h>

#include "ros/ros.h"

#include "std_msgs/Empty.h"

#include "handle_msgs/HandleSensors.h"
#include "handle_msgs/Finger.h"
#include "handle_msgs/CableTension.h"
#include "handle_msgs/HandleControl.h"

#define HAND_NAME "armH-palm-1" //"192.168.0.22"
//#define HAND_NAME "192.168.0.22" //"armH-palm-1"

// global publisther
ros::Publisher pub;

// 7200 ticks = 2pi radians
#define SPINDLE_RADIANS_TO_ENCODER_TICKS 1145.91559026 // 7200 / 2pi

void h_callback(const HandPacket& msg)
{
    handle_msgs::HandleSensors newmsg;
    newmsg.header.stamp = ros::Time(msg.stamp.tv_sec, msg.stamp.tv_usec * 1000);
    
    // for (int i=0; i<2; i++)
    // {
    //     newmsg.cableTension[i].sensor1 = msg.data.cableTension[i].sensor1;
    //     newmsg.cableTension[i].sensor2 = msg.data.cableTension[i].sensor2;
    // }
        
    for (int i=0; i<3; i++)
    {
        newmsg.fingerTactile[i].distal.resize(10);
        newmsg.fingerTactile[i].proximal.resize(12);
        for (int j=0; j<10; j++)
            newmsg.fingerTactile[i].distal[j] = msg.data.fingerTactile[i].distal[j];
        for (int j=0; j<12; j++)
            newmsg.fingerTactile[i].proximal[j] = msg.data.fingerTactile[i].proximal[j];
        
        newmsg.proximalJointAngle[i] = msg.data.proximalJointAngle[i];
        
        newmsg.distalJointAngle[i].distal.resize(4);
        newmsg.distalJointAngle[i].proximal.resize(4);
        for (int j=0; j<4; j++)
        {
            newmsg.distalJointAngle[i].distal[j] = msg.data.distalJointAngle[i].distal[j];
            newmsg.distalJointAngle[i].proximal[j] = msg.data.distalJointAngle[i].proximal[j];
        }
        
        // newmsg.fingerPVDF[i].distal.resize(3);
        // newmsg.fingerPVDF[i].proximal.resize(3);
        // for (int j=0; j<3; j++)
        // {
        //     newmsg.fingerPVDF[i].distal[j] = msg.data.fingerPVDF[i].distal[j];
        //     newmsg.fingerPVDF[i].proximal[j] = msg.data.fingerPVDF[i].proximal[j];
        // }
            
        newmsg.fingerAcceleration[i].x = msg.data.fingerAcceleration[i].x;
        newmsg.fingerAcceleration[i].y = msg.data.fingerAcceleration[i].y;
        newmsg.fingerAcceleration[i].z = msg.data.fingerAcceleration[i].z;
    }
        
    for (int i=0; i<4; i++)
    {
        newmsg.motorHallEncoder[i] = msg.data.motorHallEncoder[i];
        newmsg.motorWindingTemp[i] = msg.data.motorWindingTemp[i];
        newmsg.motorVelocity[i] = msg.data.motorVelocity[i];
        // newmsg.palmPVDF[i] = msg.data.palmPVDF[i];
    }
        
    for (int i=0; i<5; i++)
    {
        newmsg.motorHousingTemp[i] = msg.data.motorHousingTemp[i];
        newmsg.motorCurrent[i] = msg.data.motorCurrent[i];
    }
    
    for (int i=0; i<48; i++)
        newmsg.palmTactile[i] = msg.data.palmTactile[i];
    
    newmsg.airTemp = msg.data.airTemp;
    // newmsg.volts33 = msg.data.voltage.volts33;
    // newmsg.volts12 = msg.data.voltage.volts12;
    // newmsg.volts48 = msg.data.voltage.volts48;
    newmsg.fingerSpread = msg.data.fingerSpread;

    pub.publish(newmsg);
};

void calibrate_callback(const std_msgs::EmptyConstPtr& msg)
{
    HandleCommand cmd;
    cmd.calibrate = true;
    handle_send(cmd);
};

void control_callback(const handle_msgs::HandleControlConstPtr& msg)
{
    HandleCommand cmd;
    for (int i=0; i<5; i++)
    {
        switch (msg->type[i])
        {
            case handle_msgs::HandleControl::VELOCITY: 
                cmd.motorCommand[i].type = MOTOR_VELOCITY; 
                break;
            case handle_msgs::HandleControl::POSITION: 
                cmd.motorCommand[i].type = MOTOR_POSITION; 
                break;
            case handle_msgs::HandleControl::CURRENT: 
                cmd.motorCommand[i].type = MOTOR_CURRENT; 
                break;
            case handle_msgs::HandleControl::VOLTAGE: 
                cmd.motorCommand[i].type = MOTOR_VOLTAGE; 
                break;
            default:
                printf("UNKNOWN CONTROL TYPE\n");
                return;
        }
        cmd.motorCommand[i].value = msg->value[i] * SPINDLE_RADIANS_TO_ENCODER_TICKS;
        cmd.motorCommand[i].valid = msg->valid[i];
    }
    handle_send(cmd);
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "handle_ros");
    ros::NodeHandle n;
    pub = n.advertise<handle_msgs::HandleSensors>("/handle/sensors/raw", 1);
    ros::Subscriber sub1 = n.subscribe("/handle/control", 1, control_callback);
    ros::Subscriber sub2 = n.subscribe("/handle/calibrate", 1, calibrate_callback);
    
    if (handle_connect(HAND_NAME) < 0)
    {
        printf("ERROR: cannot open handle device\n");
        return -1;
    }
    
    if (handle_start(h_callback) < 0)
    {
        printf("ERROR: cannot start handle device\n");
        return -1;  
    }
    
    printf("Press CTRL-C to exit\n");
    ros::spin();
    
    if (handle_stop() < 0)
    {
        printf("WARNING: cannot stop handle thread\n");
    };
    
    return 0;
};
