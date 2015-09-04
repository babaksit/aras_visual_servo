#include "aras_visual_servo_camera/aras_visual_servo_camera.h"
VisualServoCamera::VisualServoCamera()
{
    image_transport::ImageTransport it(nh_);
    image_sub_ = it.subscribe("/labrob/camera/image_raw", 1, &VisualServoCamera::imageCB,this);
    threshold_image_pub_ = it.advertise("/aras_visual_servo/camera/thresholded_image", 1);
    camera_data_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/aras_visual_servo/camera/data",1);
    cv::namedWindow( "Orginal Image", CV_WINDOW_AUTOSIZE);
    cv::namedWindow( "Thresholded Image", CV_WINDOW_AUTOSIZE );
    cv::moveWindow("Orginal Image",0,0);
    cv::moveWindow("Thresholded Image",0,320);
}

void VisualServoCamera::imageCB(const sensor_msgs::ImageConstPtr &image_msg)
{
    try
    {
        color_image_ = cv_bridge::toCvShare(image_msg, "bgr8")->image;
        cv::cvtColor(color_image_, grey_image_, CV_BGR2GRAY);
        cv::threshold( grey_image_, threshold_image_, THRESHOLD_VALUE, MAX_BINARY_VALUE ,cv::THRESH_BINARY_INV );

        cv::Mat jacobian_mat(KERNEL_SIZE,DOF,cv::DataType<double>::type);
        cv::Mat jacobian_inverse_mat(KERNEL_SIZE,DOF,cv::DataType<double>::type);
        float kernel[4];
        calculateKernel(&threshold_image_,kernel,jacobian_mat,jacobian_inverse_mat);
        publishCameraData(kernel,jacobian_inverse_mat);

        cv::imshow("Orginal Image",color_image_);
        cv::imshow("Thresholded Image",threshold_image_);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_msg->encoding.c_str());
    }

}

bool VisualServoCamera::calculateKernel(const cv::Mat *image, float kernel[KERNEL_SIZE], cv::Mat &jacobian_mat , cv::Mat &jacobian_inverse_mat)
{
    //we should add mutex-lock
    if(jacobian_mat.rows !=KERNEL_SIZE && jacobian_mat.cols!=DOF)
    {
        ROS_ERROR("the jacobian matrix size is not %d x %d",KERNEL_SIZE,DOF);
        return false;
    }
    //just in case
    for(int i=0;i<KERNEL_SIZE;i++)
    {
        for(int j=0;j<DOF;j++)
        {
            jacobian_mat.at<double>(i,j)=0;
        }
    }

    jacobian_mat.at<double>(3,3)=-1;

    int img_height = image->rows;
    int img_width = image->cols;
    double jxx,jxtheta,jyy,jytheta;

    for(int i=0;i<KERNEL_SIZE;i++)
    {
        kernel[i] = 0;
    }

    double kx = 0;
    double ky = 0;

    for(int i=0;i<img_height;i++ )
    {
        kx = pow(((i+1)*DELTA_X),KERNEL_P);//k_x(x,y) = x^(-2);
        jxx =  KERNEL_P * pow(((i+1)*DELTA_X),(KERNEL_P-1));//dk_x(x,y)/dx = -2 * x^(-3)
        for(int j=0;j<img_width;j++)
        {
            double pixel_value = (double)image->at<uchar>(i,j)/MAX_BINARY_VALUE;

            ky = pow(((j+1)*DELTA_Y),KERNEL_Q);

            jyy = KERNEL_Q * pow(((j+1)*DELTA_Y),(KERNEL_Q-1));
            jxtheta = jxx*(j + 1)*DELTA_Y;
            jytheta = jyy*(i + 1)*DELTA_X;
            kernel[0] = kernel[0] + kx * pixel_value*DELTA_Y*DELTA_X;
            kernel[1] = kernel[1] + ky * pixel_value*DELTA_Y*DELTA_X;
            kernel[2] = kernel[2] + pixel_value*DELTA_Y*DELTA_X;

            jacobian_mat.at<double>(0,0) = jacobian_mat.at<double>(0,0) - jxx * pixel_value; //jacobianxx
            jacobian_mat.at<double>(0,3) = jacobian_mat.at<double>(0,3) + jxtheta * pixel_value;//jacobianxtheta
            jacobian_mat.at<double>(1,1) = jacobian_mat.at<double>(1,1) - jyy * pixel_value;//jacobianyy
            jacobian_mat.at<double>(1,3) = jacobian_mat.at<double>(1,3) - jytheta * pixel_value;//jacobianytheta
            jacobian_mat.at<double>(2,2) = jacobian_mat.at<double>(2,2) + 2 * pixel_value;//jacobianz

        }
    }

    cv::Moments image_moments = cv::moments(*image);
    kernel[3] = 0.5 * atan2( (2*image_moments.mu11/255.0) , (image_moments.mu20/255.0-image_moments.mu02/255.0) );
    cv::invert(jacobian_mat,jacobian_inverse_mat);
}

void VisualServoCamera::publishCameraData(const float kernel[KERNEL_SIZE] ,const cv::Mat jacobian_inverse_mat)
{
    std_msgs::Float64MultiArray camera_data_msg;
    for(int i=0;i<jacobian_inverse_mat.rows;i++)
    {
        for(int j=0;j<jacobian_inverse_mat.cols;j++)
        {
            camera_data_msg.data.push_back(jacobian_inverse_mat.at<double>(i,j));
        }
    }
    for(int i=0;i<KERNEL_SIZE;i++)
    {
        camera_data_msg.data.push_back(kernel[i]);
    }
    camera_data_pub_.publish(camera_data_msg);
}

VisualServoCamera::~VisualServoCamera()
{

}
