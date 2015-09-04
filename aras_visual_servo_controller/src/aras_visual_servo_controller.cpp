#include "aras_visual_servo_controller/aras_visual_servo_controller.h"
VisualServoController::VisualServoController():jacobian_inverse_mat_(KERNEL_SIZE,DOF, cv::DataType<double>::type)
{
    initializeSubscribers();
    initializePublishers();
}

bool VisualServoController::setJointPositions(float target_positions[JOINTS_NUM])
{
    for(int i=0;i<JOINTS_NUM;i++)
    {
        std_msgs::Float64 msg;
        msg.data=target_positions[i];
        joint_pub_[i].publish(msg);
    }
    return true;
}

float *VisualServoController::getJointPositions()
{
    return joints_position_;
}

void VisualServoController::setTargetPositions(float target_positions[JOINTS_NUM])
{

    hardSetJointPosition(target_positions);
    camera_call_backed_ =false;
    while(camera_call_backed_ == false)
    {
        ros::spinOnce();
    }
    ROS_INFO("camera call backed");
    for(int i=0;i<KERNEL_SIZE;i++)
    {
        target_kernel_[i] = kernel_[i];
    }
    for(int i=0;i<KERNEL_SIZE;i++)
    {
        ROS_INFO("kernel target %d = %lf" ,i, kernel_[i]);
    }

}

void VisualServoController::setInitialPositions(float target_positions[])
{
    hardSetJointPosition(target_positions);
    camera_call_backed_ =false;
    while(camera_call_backed_ == false)
    {
        ros::spinOnce();
    }
    for(int i=0;i<KERNEL_SIZE;i++)
    {
        ROS_INFO("kernel initial %d = %lf" ,i, kernel_[i]);
    }
}

void VisualServoController::hardSetJointPosition(float target_positions[JOINTS_NUM])
{
    bool close_enough=true;

    //TODO set timeout

    while(true)
    {
        setJointPositions(target_positions);
        ros::spinOnce();
        for(int i=0;i<JOINTS_NUM;i++)
        {
            if(fabs((target_positions[i]-joints_position_[i]))>JOINTS_ERROR)
            {
                close_enough =false;
//                ROS_INFO("%lf %d",fabs((target_positions[i]-joints_position_[i])),i);
//                ROS_INFO("%lf , %lf",target_positions[i],joints_position_[i]);
                break;
            }
        }
        if(close_enough == true)
        {
            return;
        }
        close_enough = true;
//        ROS_INFO("not close enough");
    }

    return;
}

void VisualServoController::initializeSubscribers()
{
    joint_state_sub_ = nh_.subscribe("/aras_visual_servo/joint_states", 1, &VisualServoController::jointStateCB,this);
    camera_data_sub_ = nh_.subscribe("/aras_visual_servo/camera/data" , 1, &VisualServoController::cameraDataCB,this);

}



void VisualServoController::initializePublishers()
{
    joint_pub_[0]=nh_.advertise<std_msgs::Float64>("/aras_visual_servo/gantry_position_controller/command",10);
    //because the joint 1 is constant we don't need to set it's position.
    for(int i=1;i<JOINTS_NUM;i++)
    {
        int k=i;
        if(i>=4)
            k++;
        std::stringstream pub_topic;
        pub_topic << "/aras_visual_servo/joint" << k << "_position_controller/command";
        joint_pub_[i]=nh_.advertise<std_msgs::Float64>(pub_topic.str().c_str(),10);
    }

}

void VisualServoController::jointStateCB(const sensor_msgs::JointStatePtr& joint_states)
{
    for(int i=0;i<JOINTS_NUM-1;i++)
    {
        joints_position_[i+1]= joint_states->position[i];
    }
    joints_position_[0] = joint_states->position[JOINTS_NUM-1];
}

void VisualServoController::cameraDataCB(const std_msgs::Float64MultiArray::ConstPtr &camera_data_arr)
{

    if(camera_data_arr->data.size() != CAMERA_DATA_SIZE)
    {
        ROS_ERROR("Camera Data Size is not %d" , CAMERA_DATA_SIZE);
        return;
    }
    camera_call_backed_ = true;
    for(int i=0;i<KERNEL_SIZE;i++)
    {
        for(int j=0;j<DOF;j++)
        {
            jacobian_inverse_mat_.at<double>(j,i) = camera_data_arr->data[(i*DOF+j)];
        }
    }
    for(int i=CAMERA_DATA_SIZE-KERNEL_SIZE;i<CAMERA_DATA_SIZE;i++)
    {
        kernel_[i-(CAMERA_DATA_SIZE-KERNEL_SIZE)] = camera_data_arr->data[i];
    }
}

void VisualServoController::executeControlAlgorithm()
{
    while(ros::ok())
    {
//        ROS_INFO("executeControlAlgorithm");
        float velocity_x = 0, velocity_y = 0;


        cv::Mat error(KERNEL_SIZE,1, cv::DataType<double>::type);
        cv::Mat control_signal(DOF,1, cv::DataType<double>::type);
        //!!!!!!!!yekbar etefagh biofad
        cv::Mat lambda(KERNEL_SIZE,KERNEL_SIZE, cv::DataType<double>::type);
        //TODO
        //replace it with memcpy

        for(int i=0;i<KERNEL_SIZE;i++)
        {
            for(int j=0;j<KERNEL_SIZE;j++)
            {
                lambda.at<double>(i,j) = 0;
            }
        }
        lambda.at<double>(0,0) = LAMBDA_X;
        lambda.at<double>(1,1) = LAMBDA_Y;
        lambda.at<double>(2,2) = LAMBDA_Z;
        lambda.at<double>(3,3) = LAMBDA_THETA;


        for(int i=0;i<KERNEL_SIZE;i++)
        {
            error.at<double>(i,0) = kernel_[i] - target_kernel_[i] ;
        }

        control_signal = -1 * lambda * jacobian_inverse_mat_ * error;
        control_signal.at<double>(0,0) *= -1 ;

        velocity_x =  control_signal.at<double>(0,0);
        velocity_y =  control_signal.at<double>(1,0);
        control_signal.at<double>(0,0) = velocity_x*cos(((double)joints_position_[1] + (double)joints_position_[5]))
                - velocity_y*sin(((double)joints_position_[1] + (double)joints_position_[5]));
        control_signal.at<double>(1,0) = velocity_x*sin(((double)joints_position_[1] + (double)joints_position_[5]))
                + velocity_y*cos(((double)joints_position_[1] + (double)joints_position_[5]));
        float current_x,current_y,current_z,current_yaw;
        float desired_x,desired_y,desired_z,desired_yaw;

        //Calcute current position
        forwardKinematic(joints_position_, current_x, current_y, current_z, current_yaw);
//        for(int i=0;i<DOF;i++)
//        {
//            ROS_INFO("error :%lf",error.at<double>(i,0));
//        }

        //Integral velocity to convert to Position;
        desired_x = current_x + control_signal.at<double>(0,0)*SAMPLE_TIME;
        desired_y = current_y + control_signal.at<double>(1,0)*SAMPLE_TIME;
        desired_z = current_z + control_signal.at<double>(2,0)*SAMPLE_TIME;
        desired_yaw = current_yaw + control_signal.at<double>(3,0);

        float target_position[JOINTS_NUM];
        // Compute Target joints value(Inverse Kinematic)

        inverseKinematic(target_position, desired_x, desired_y, desired_z, desired_yaw);

        hardSetJointPosition(target_position);
        camera_call_backed_ =false;
        while(camera_call_backed_ == false)
        {
            ros::spinOnce();
        }
    }
}

void VisualServoController::inverseKinematic(float target_joints[JOINTS_NUM], float x, float y, float z, float a)
{
    float delta, c_theta3, s_theta3, c_theta2, s_theta2;
    delta = z - L5 - L1;
    c_theta3 = (x*x + delta*delta - L2*L2 - L3*L3)/(2*L2*L3);
    s_theta3 = - sqrt(1 - c_theta3*c_theta3);
    c_theta2 = (L3*s_theta3*x + (L2 + L3*c_theta3)*delta)/(L2*L2 + L3*L3 + 2*L2*L3*c_theta3);
    s_theta2 = ((L2 + L3*c_theta3)*x - L3*s_theta3*delta)/(L2*L2 + L3*L3 + 2*L2*L3*c_theta3);
    target_joints[0] = y;//gantry
    target_joints[1] = 0; //theta1 is zero
    target_joints[2] = atan2(s_theta2, c_theta2); //theta2, Degree
    target_joints[3] = atan2(s_theta3, c_theta3); //theta3, Degree
    target_joints[4] =  -(target_joints[2] + target_joints[3]); //theta5 = -(theta2 + theta3), Degree
    target_joints[5] = a; //theta 1 is zero, so A change only with theta6
}
void VisualServoController::forwardKinematic(float target_joints[JOINTS_NUM], float &x, float &y, float &z, float &a)
{
    float theta1 = target_joints[1];
    float theta2 = target_joints[2];
    float theta3 = target_joints[3];
    float theta5 = target_joints[4];
    float theta6 = target_joints[5];
    x = cos(theta1)*(L2*sin(theta2) + L3*sin(theta2 + theta3) + L5*sin(theta2 + theta3 + theta5));//m
    y = sin(theta1)*(L2*sin(theta2) + L3*sin(theta2 + theta3) + L5*sin(theta2 + theta3 + theta5)) + (target_joints[0]);//m
    z = L1 + L2*cos(theta2) + L3*cos(theta2 + theta3) + L5*cos(theta2 + theta3 + theta5);//m
    a = (theta1 + theta6);//radian
}
VisualServoController::~VisualServoController()
{

}
