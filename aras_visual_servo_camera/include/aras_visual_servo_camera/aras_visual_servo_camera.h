#ifndef VISUALSERVOCAMERA_H
#define VISUALSERVOCAMERA_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <strstream>
#include <sstream>
#include <exception>

#include "image_transport/subscriber.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64MultiArray.h>


#define MAX_BINARY_VALUE 255.0
#define THRESHOLD_VALUE 250.0

#define DELTA_X 0.017
#define DELTA_Y 0.0185

#define KERNEL_P -2
#define KERNEL_Q -2

#define KERNEL_SIZE 4
#define DOF 4


class VisualServoCamera
{
public:
    VisualServoCamera();
    ~VisualServoCamera();
private:


    ros::NodeHandle nh_;
    void imageCB(const sensor_msgs::ImageConstPtr& image_msg);

    cv::Mat cur_image_; //current image
    cv::Mat grey_image_, color_image_,threshold_image_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher threshold_image_pub_;
    cv_bridge::CvImage threshold_image_msg_;// thresholded image

    ros::Publisher camera_data_pub_;

    // Kernel gets image and computes kesi, Ger and Jacobian
    bool calculateKernel(const cv::Mat *image,float kernel[KERNEL_SIZE],cv::Mat &jacobian_mat , cv::Mat &jacobian_inverse_mat);
    void publishCameraData(const float kernel[], const cv::Mat jacobian_inverse_mat);




};

#endif
