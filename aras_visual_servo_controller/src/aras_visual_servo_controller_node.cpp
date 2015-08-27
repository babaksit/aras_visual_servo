#include <aras_visual_servo_controller/aras_visual_servo_controller.h>
#include <ros/ros.h>


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"aras_visual_servo_controller");
    VisualServoController *visual_servo_controller = new VisualServoController();
    float joint_positions[6];
    joint_positions[0]=0;
    joint_positions[1]=0;
    joint_positions[2]=0;
    joint_positions[3]=0;
    joint_positions[4]=0;
    joint_positions[5]=0;
    sleep(5);
    visual_servo_controller->setTargetPositions(joint_positions);

    joint_positions[0]=0;
    joint_positions[1]=0;
    joint_positions[2]=20.0*M_PI/180.0;
    joint_positions[3]=-10.0*M_PI/180.0;
    joint_positions[4]=-10.0*M_PI/180.0;
    joint_positions[5]=0;
    visual_servo_controller->setInitialPositions(joint_positions);
    visual_servo_controller->executeControlAlgorithm();
    delete visual_servo_controller;
    return 0;
}
