#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <ros/console.h>
#include <string>

class TeleopJoy
{
public:
    TeleopJoy();
    
private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
    
    ros::NodeHandle nh_;

    int linear_x_{},angular_z_{};
    double lin_x_scale_{0.2}, ang_z_scale_{0.2};

    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
    
};

TeleopJoy::TeleopJoy() : linear_x_(1), angular_z_(3) //change 3 from 2 for angular_z_(x)
{
    nh_.param("axis_linear_x", linear_x_, linear_x_);
    nh_.param("axis_angular_z", angular_z_, angular_z_);

    nh_.param("scale_linear_x", lin_x_scale_, lin_x_scale_);
    nh_.param("scale_angular_z", ang_z_scale_, ang_z_scale_);

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::joyCallback, this);
}

void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    geometry_msgs::Twist twist;

    if (joy->buttons[4] == 1)
    {

        
        if (joy->buttons[5] == 1)
        {
            //slow
            lin_x_scale_ = 0.5;
            ang_z_scale_ = -0.5;
            
        }
        else if (joy->buttons[7] == 1)
        {
            //fast
            lin_x_scale_ = 1.0;
            ang_z_scale_ = -1.0;
            
        }
        else
        {
            //normal
            lin_x_scale_ = 0.2;
            ang_z_scale_ = -0.2;
           
            
        }
    }
    
    else
    {
        lin_x_scale_ = 0.0;
        ang_z_scale_ = 0.0;
    }
    


    twist.linear.x = lin_x_scale_ * joy->axes[linear_x_];
    twist.angular.z = ang_z_scale_ * joy->axes[angular_z_];
    

   ROS_INFO("linear_x: [%f], angular_z: [%f]", twist.linear.x , twist.angular.z);
    vel_pub_.publish(twist);

}

int main(int argc, char **argv)
{
    
    //ros::Rate loop_rate(10);
    ROS_INFO("Init Node");
    ros::init(argc, argv, "puru_joystick_node");
    TeleopJoy teleop_joy;
    ros::spin();
    //loop_rate.sleep();
}
