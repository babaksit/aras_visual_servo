#include <aras_visual_servo_camera/aras_visual_servo_camera.h>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"aras_visual_servo_camera");
    VisualServoCamera *visual_servo_camera = new VisualServoCamera();
    ros::spin();
    delete visual_servo_camera;
    return 0;
}
