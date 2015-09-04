#include <aras_visual_servo_controller/aras_visual_servo_controller.h>
#include <ros/ros.h>
#include "csignal"

void signalHandler( int signum )
{
    ROS_INFO("Interrupt signal (%d) received.\n",signum);
    exit(signum);
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"aras_visual_servo_controller");
    VisualServoController *visual_servo_controller = new VisualServoController();
    signal(SIGINT,signalHandler);

    float joint_positions[6];
    joint_positions[0]=0;
    joint_positions[1]=0;
    joint_positions[2]=0;
    joint_positions[3]=0;
    joint_positions[4]=0;
    joint_positions[5]=0;

    visual_servo_controller->setTargetPositions(joint_positions);

    joint_positions[0]=0.1;
    joint_positions[1]=0;
    joint_positions[2]=10.0*M_PI/180.0;
    joint_positions[3]=-5.0*M_PI/180.0;
    joint_positions[4]=-5.0*M_PI/180.0;
    joint_positions[5]=0;
    visual_servo_controller->setInitialPositions(joint_positions);
    visual_servo_controller->executeControlAlgorithm();
    delete visual_servo_controller;
    return 0;
}
