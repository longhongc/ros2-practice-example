#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals; 

// getting the install/share path
std::string package_share_directory = ament_index_cpp::get_package_share_directory("sensors");

class LaserPublisher: public rclcpp::Node{
    public:
        LaserPublisher()
        :Node("laser_publisher"),
         storage_options({package_share_directory + "/laser_test_data", "sqlite3"}),
         pub_rate_{80}{
            std::string format_str = "cdr"; 
            converter_options.input_serialization_format = format_str;
            converter_options.output_serialization_format = format_str; 

            // rosbag reader
            reader->open(storage_options, converter_options);

            publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10); 
            // convert frequency to interval
            auto interval = std::chrono::milliseconds(1000/pub_rate_); 
            timer_ = this->create_wall_timer(interval, std::bind(&LaserPublisher::timer_callback, this)); 
         }
    private:
        void timer_callback(){
            if(reader->has_next()){
                auto bag_message = reader->read_next();

                rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data); 
                auto message = sensor_msgs::msg::LaserScan(); 
                serialization.deserialize_message(&extracted_serialized_msg, &message);

                auto sec = message.header.stamp.sec;
                auto nsec = message.header.stamp.nanosec;

                // print the laser timestamp to examine the subscriber
                RCLCPP_INFO(this->get_logger(), "Publishing laser %d.%ld", sec, nsec); 
                publisher_->publish(message); 
            }else{
                // restart the rosbag reader to read again
                RCLCPP_INFO(this->get_logger(), "Repeat data again"); 
                reader.reset(); 
                reader = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
                reader->open(storage_options, converter_options);
            }
  
        }

        const rosbag2_cpp::StorageOptions storage_options;
        rosbag2_cpp::ConverterOptions converter_options; 
        std::unique_ptr<rosbag2_cpp::readers::SequentialReader> reader = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
        rclcpp::Serialization<sensor_msgs::msg::LaserScan> serialization;

        int pub_rate_; //Hz
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
}; 

int main(int argc, char ** argv){
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<LaserPublisher>()); 
    rclcpp::shutdown(); 

    return 0; 
}
