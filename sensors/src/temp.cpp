#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_interfaces/msg/temperature.hpp"

using namespace std::chrono_literals; 

class TemperaturePublisher: public rclcpp::Node{
    public:
        TemperaturePublisher()
        :Node("temperature_publisher"),
         pub_rate_{30}, 
         count_{0}{
            publisher_ = this->create_publisher<sensor_interfaces::msg::Temperature>("temp", 10); 
            auto interval = std::chrono::milliseconds(1000/pub_rate_); 
            timer_ = this->create_wall_timer(interval, std::bind(&TemperaturePublisher::timer_callback, this)); 
        }
    private:
        void timer_callback(){
            auto message = sensor_interfaces::msg::Temperature(); 
            message.temp = count_++;
            RCLCPP_INFO(this->get_logger(), "Publishing temperature %lf", message.temp); 
            publisher_->publish(message); 
        }
        int pub_rate_; //Hz
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::Publisher<sensor_interfaces::msg::Temperature>::SharedPtr publisher_; 
        size_t count_; 
}; 

int main(int argc,char **argv){
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<TemperaturePublisher>()); 
    rclcpp::shutdown(); 

    return 0; 
}
