#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals; 

class SpeedPublisher: public rclcpp::Node{
    public:
        SpeedPublisher()
        :Node("speed_publisher"),
         pub_rate_{35}{ 
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10); 
            auto interval = std::chrono::milliseconds(1000/pub_rate_); 
            timer_ = this->create_wall_timer(interval, std::bind(&SpeedPublisher::timer_callback, this)); 
        }
    private:
        double create_random_error(int mean, int sigma){
            std::random_device rd;
            std::mt19937 gen(rd()); 
            std::normal_distribution<double> gauss_dist(mean, sigma); //velocity error, mean=0 m/s, sigma=2
            return gauss_dist(gen); 
        }

        void timer_callback(){
            auto message = geometry_msgs::msg::Twist(); 

            //simulate 2D turtlebot cmd_vel with gaussian measure error
            message.linear.x = 2 + create_random_error(0, 2); 
            message.linear.y = 0; 
            message.linear.z = 0; 
            message.angular.x = 0; 
            message.angular.y = 0; 
            message.angular.z = 1 + create_random_error(0, 1); 

            auto lx = message.linear.x; 
            auto ly = message.linear.y; 
            auto lz = message.linear.z; 
            auto ax = message.angular.x; 
            auto ay = message.angular.y; 
            auto az = message.angular.z; 

            RCLCPP_INFO(this->get_logger(), "Publishing speed "
                        "linear:[x: %lf, y: %lf, z: %lf]" 
                        " angular:[x: %lf, y: %lf, z: %lf]", lx, ly, lz, ax, ay, az); 

            publisher_->publish(message); 
        }

        int pub_rate_; //Hz
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; 
}; 

int main(int argc,char **argv){
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<SpeedPublisher>()); 
    rclcpp::shutdown(); 

    return 0; 
}
