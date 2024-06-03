#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/int16.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <libserial/SerialPort.h>
#include <chrono>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_broadcaster.h"



using namespace std::chrono_literals;

using std::placeholders::_1;

class WaspTransmission: public rclcpp::Node
{
public:
    WaspTransmission(): Node("wasp_transmisson_node")
    {
        declare_parameter<std::string>("port","/dev/ttyACM1");
        std::string port_ = get_parameter("port").as_string();
        vel_sub_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel",10,std::bind(&WaspTransmission::velCallback, this, _1));
        timer_ = create_wall_timer(10ms, std::bind(&WaspTransmission::timerCallback, this));
        tick1_pub_ = create_publisher<std_msgs::msg::Int64>("motor/tick1", 10);
        tick2_pub_ = create_publisher<std_msgs::msg::Int64>("motor/tick2", 10);

        rpm1_pub_ = create_publisher<std_msgs::msg::Int16>("motor/rpm1", 10);  //Actual Speed
        rpm2_pub_ = create_publisher<std_msgs::msg::Int16>("motor/rpm2", 10);

        odom_raw_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom/raw", 10);
        odom_ekf_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        mbed_.Open(port_);
        mbed_.SetBaudRate(LibSerial::BaudRate::BAUD_2000000);
      
    }

    ~WaspTransmission()
    {
        mbed_.Close();
    }

    void velCallback(const geometry_msgs::msg::Twist &msg)
    {
       
        float v = msg.linear.x;
        float w = msg.angular.z;
        mbed_.Write("DYNV:"+ std::to_string(v)+"\n");
        mbed_.Write("DYNW:"+ std::to_string(w)+"\n");
        //RCLCPP_INFO_STREAM(get_logger(), "I hearded: "<< msg.data.c_str());
    }

    std::string trimEnd(const std::string& str) 
    {
        std::string result = str;
        if(!result.empty() && result.back() == '\n')
        {   
            result.pop_back();
        }

        if(!result.empty() && result.back() == '\r')
        {   
            result.pop_back();
        }
      
        return result;
    }

    std::vector<std::string> splitString(const std::string& str, char delimiter) 
    {
        std::vector<std::string> tokens;
        std::stringstream ss(str);
        std::string token;
        while (std::getline(ss, token, delimiter)) {
            tokens.push_back(token);
        }
        return tokens;
    }

    struct Quaternion 
    {
        double w, x, y, z;
    };

    Quaternion eulerToQuaternion(double roll, double pitch, double yaw) 
    {
        // Calculate half angles
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        Quaternion q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;

        return q;
    }

    void timerCallback()
    {
        

        std::string readData;
        if(rclcpp::ok() && mbed_.IsDataAvailable())
        {
            mbed_.ReadLine(readData);
            readData = trimEnd(readData);
            
            if (readData.find("MENC:") == 0) 
            {
                int64_t left_enc, right_enc;
                std::vector<std::string> parts = splitString(readData, ':');
            
                if (parts.size() == 3) {
                    // Extract the number parts
                    std::string part1 = parts[1];
                    std::string part2 = parts[2];

                    // Convert the strings to integers
                    left_enc = std::stoi(part1);
                    right_enc = std::stoi(part2);
                    //std::cout << "Left Encoder: " << left_enc << ", Right Encoder: " << right_enc << std::endl;
                }

                auto tick1_msg = std_msgs::msg::Int64();
                tick1_msg.data = left_enc;
                tick1_pub_->publish(tick1_msg);

                auto tick2_msg = std_msgs::msg::Int64();
                tick2_msg.data = right_enc;
                tick2_pub_->publish(tick2_msg);

            }

            if (readData.find("MASP:") == 0) 
            {
                int32_t rpm1, rpm2;
                std::vector<std::string> parts = splitString(readData, ':');
            
                if (parts.size() == 3) {
                    // Extract the number parts
                    std::string part1 = parts[1];
                    std::string part2 = parts[2];

                    // Convert the strings to integers
                    rpm1 = std::stoi(part1);
                    rpm2 = std::stoi(part2);
                    //std::cout << "Left Encoder: " << left_enc << ", Right Encoder: " << right_enc << std::endl;
                }

                auto rpm1_msg = std_msgs::msg::Int16();
                rpm1_msg.data = rpm1;
                rpm1_pub_->publish(rpm1_msg);

                auto rpm2_msg = std_msgs::msg::Int16();
                rpm2_msg.data = rpm2;
                rpm2_pub_->publish(rpm2_msg);

            }

            if (readData.find("RODM:") == 0) 
            {
                double odom_x, odom_y, odom_yaw;
                std::vector<std::string> parts = splitString(readData, ':');
            
                if (parts.size() == 4) {
                    // Extract the number parts
                    std::string part1 = parts[1];
                    std::string part2 = parts[2];
                    std::string part3 = parts[3];

                    // Convert the strings to integers
                    odom_x = std::stof(part1);
                    odom_y = std::stof(part2);
                    odom_yaw = std::stof(part3);

                    /*std::cout << "Quaternion: "
                    << "x = " << part1 << ", "
                    << "y = " << part2 << ", "
                    << "yaw = " << part3 << std::endl;*/
                    
                }

                auto odom_raw_msg = nav_msgs::msg::Odometry();
                odom_raw_msg.header.stamp = this->get_clock()->now();
                odom_raw_msg.header.frame_id = "odom";
                odom_raw_msg.child_frame_id = "base_link";
                odom_raw_msg.pose.pose.position.x = odom_x;
                odom_raw_msg.pose.pose.position.y = odom_y;
                odom_raw_msg.pose.pose.position.z = 0.0;

                double roll = 0.0;  // rotation around x-axis in radians
                double pitch = 0.0; // rotation around y-axis in radians
                double yaw = odom_yaw;   // rotation around z-axis in radians

                Quaternion q = eulerToQuaternion(roll, pitch, yaw);
                odom_raw_msg.pose.pose.orientation.w = q.w;
                odom_raw_msg.pose.pose.orientation.x = q.x;
                odom_raw_msg.pose.pose.orientation.y = q.y;
                odom_raw_msg.pose.pose.orientation.z = q.z;

                odom_raw_msg.pose.covariance[0] = 0.001;
		        odom_raw_msg.pose.covariance[7] = 0.001;
		        odom_raw_msg.pose.covariance[35] = 0.001;
		
		        odom_raw_msg.twist.covariance[0] = 0.0001;
		        odom_raw_msg.twist.covariance[7] = 0.0001;
		        odom_raw_msg.twist.covariance[35] = 0.0001;

                odom_raw_pub_->publish(odom_raw_msg);

                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = this->get_clock()->now();
                t.header.frame_id = "odom";
                t.child_frame_id = "base_link";

                
                t.transform.translation.x = odom_x;
                t.transform.translation.y = odom_y;
                t.transform.translation.z = 0.0;
                t.transform.rotation.x = q.x;
                t.transform.rotation.y = q.y;
                t.transform.rotation.z = q.z;
                t.transform.rotation.w = q.w;

                // Send the transformation
                //tf_broadcaster_->sendTransform(t);
                

            }

            if (readData.find("FODM:") == 0) 
            {
                double fodom_x, fodom_y, fodom_yaw;
                std::vector<std::string> parts = splitString(readData, ':');
            
                if (parts.size() == 4) {
                    // Extract the number parts
                    std::string part1 = parts[1];
                    std::string part2 = parts[2];
                    std::string part3 = parts[3];

                    // Convert the strings to integers
                    fodom_x = std::stof(part1);
                    fodom_y = std::stof(part2);
                    fodom_yaw = std::stof(part3);

                    /*std::cout << "Quaternion: "
                    << "x = " << part1 << ", "
                    << "y = " << part2 << ", "
                    << "yaw = " << part3 << std::endl;*/
                    
                }

                auto odom_f_msg = nav_msgs::msg::Odometry();
                odom_f_msg.header.stamp = this->get_clock()->now();
                odom_f_msg.header.frame_id = "odom";
                odom_f_msg.child_frame_id = "base_link";
                odom_f_msg.pose.pose.position.x = fodom_x;
                odom_f_msg.pose.pose.position.y = fodom_y;
                odom_f_msg.pose.pose.position.z = 0.0;

                double froll = 0.0;  // rotation around x-axis in radians
                double fpitch = 0.0; // rotation around y-axis in radians
                double fyaw = fodom_yaw;   // rotation around z-axis in radians

                Quaternion fq = eulerToQuaternion(froll, fpitch, fyaw);
                odom_f_msg.pose.pose.orientation.w = fq.w;
                odom_f_msg.pose.pose.orientation.x = fq.x;
                odom_f_msg.pose.pose.orientation.y = fq.y;
                odom_f_msg.pose.pose.orientation.z = fq.z;

                odom_f_msg.pose.covariance[0] = 0.001;
		        odom_f_msg.pose.covariance[7] = 0.001;
		        odom_f_msg.pose.covariance[35] = 0.001;
		
		        odom_f_msg.twist.covariance[0] = 0.0001;
		        odom_f_msg.twist.covariance[7] = 0.0001;
		        odom_f_msg.twist.covariance[35] = 0.0001;

                odom_ekf_pub_->publish(odom_f_msg);

                geometry_msgs::msg::TransformStamped ft;
                ft.header.stamp = this->get_clock()->now();
                ft.header.frame_id = "odom";
                ft.child_frame_id = "base_link";

                
                ft.transform.translation.x = fodom_x;
                ft.transform.translation.y = fodom_y;
                ft.transform.translation.z = 0.0;
                ft.transform.rotation.x = fq.x;
                ft.transform.rotation.y = fq.y;
                ft.transform.rotation.z = fq.z;
                ft.transform.rotation.w = fq.w;

                // Send the transformation
                tf_broadcaster_->sendTransform(ft);
                

            }

            if (readData.find("IMUS:") == 0) 
            {
                double ax, ay, az,gx,gy,gz,qw,qx,qy,qz;
                std::vector<std::string> parts = splitString(readData, ':');
            
                if (parts.size() == 11) {
                    // Extract the number parts
                    std::string part1 = parts[1];
                    std::string part2 = parts[2];
                    std::string part3 = parts[3];

                    std::string part4 = parts[4];
                    std::string part5 = parts[5];
                    std::string part6 = parts[6];

                    std::string part7 = parts[7];
                    std::string part8 = parts[8];
                    std::string part9 = parts[9];
                    std::string part10 = parts[10];

                    // Convert the strings to integers
                    ax = std::stof(part1);
                    ay = std::stof(part2);
                    az = std::stof(part3);

                    gx = std::stof(part4);
                    gy = std::stof(part5);
                    gz = std::stof(part6);

                    qw = std::stof(part7);
                    qx = std::stof(part8);
                    qy = std::stof(part9);
                    qz = std::stof(part10);

                    /*std::cout << "Quaternion: "
                    << "x = " << part1 << ", "
                    << "y = " << part2 << ", "
                    << "yaw = " << part3 << std::endl;*/
                    
                }

                auto imu_msg = sensor_msgs::msg::Imu();
                imu_msg.header.stamp = this->get_clock()->now();
                imu_msg.header.frame_id = "imu_link";

                imu_msg.angular_velocity.x = gx;
                imu_msg.angular_velocity.y = gy;
                imu_msg.angular_velocity.z = gz;

                imu_msg.linear_acceleration.x = ax;
                imu_msg.linear_acceleration.y = ay;
                imu_msg.linear_acceleration.z = az;

                imu_msg.orientation.w = qw;
                imu_msg.orientation.x = qx;
                imu_msg.orientation.y = qy;
                imu_msg.orientation.z = qz;

                imu_msg.angular_velocity_covariance[0] = 0.0001;
                imu_msg.angular_velocity_covariance[4] = 0.0001;
                imu_msg.angular_velocity_covariance[8] = 0.00001;

                imu_msg.linear_acceleration_covariance[0] = 0.0001;
                imu_msg.linear_acceleration_covariance[4] = 0.0001;
                imu_msg.linear_acceleration_covariance[8] = 0.00001;

                imu_pub_->publish(imu_msg);

               
                

            }
        }
    
    }
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr tick1_pub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr tick2_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr rpm1_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr rpm2_pub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_raw_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_ekf_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr timer_;
    LibSerial::SerialPort mbed_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaspTransmission>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
