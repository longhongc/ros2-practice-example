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

#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals; 

class SpeedPublisher: public rclcpp::Node{
    public:
        SpeedPublisher()
        :Node("speed_publisher"),
         storage_options({"rosbag2_test_data", "sqlite3"}),
         pub_rate_{35}{
            std::string format_str = "cdr"; 
            converter_options.input_serialization_format = format_str;
            converter_options.output_serialization_format = format_str; 

            reader->open(storage_options, converter_options);

            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10); 
            auto interval = std::chrono::milliseconds(1000/pub_rate_); 
            timer_ = this->create_wall_timer(interval, std::bind(&SpeedPublisher::timer_callback, this)); 
         }
    private:
        void timer_callback(){
            if(reader->has_next()){
                auto bag_message = reader->read_next();

                rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data); 
                auto message = geometry_msgs::msg::Twist(); 
                serialization.deserialize_message(&extracted_serialized_msg, &message);
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
            }else{
                RCLCPP_INFO(this->get_logger(), "Data end, exiting..."); 
                rclcpp::shutdown(); 
            }
  
        }

        const rosbag2_cpp::StorageOptions storage_options;
        rosbag2_cpp::ConverterOptions converter_options; 
        std::unique_ptr<rosbag2_cpp::readers::SequentialReader> reader = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
        rclcpp::Serialization<geometry_msgs::msg::Twist> serialization;

        int pub_rate_; //Hz
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
}; 

int main(int argc, char ** argv){
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<SpeedPublisher>()); 
    rclcpp::shutdown(); 

    return 0; 
}

// int main(int argc, char ** argv)
// {
//     const rosbag2_cpp::StorageOptions storage_options({"rosbag2_test_data", "sqlite3"});
//     rosbag2_cpp::ConverterOptions converter_options{}; 
//
//     std::string format_str = "cdr"; 
//     converter_options.input_serialization_format = format_str;
//     converter_options.output_serialization_format = format_str; 
//
//     std::unique_ptr<rosbag2_cpp::readers::SequentialReader> reader = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
//
//     reader->open(storage_options, converter_options);
//
//     rclcpp::Serialization<geometry_msgs::msg::Twist> serialization;
//     std::vector<std::string> topics;
//     int count=0; 
//     while (reader->has_next()) {
//       auto bag_message = reader->read_next();
//       topics.push_back(bag_message->topic_name);
//
//       geometry_msgs::msg::Twist extracted_test_msg; 
//       rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
//       serialization.deserialize_message(
//         &extracted_serialized_msg, &extracted_test_msg);
//       std::cout << "x: " << extracted_test_msg.linear.x; 
//       std::cout << "count: " << count++ << std::endl; 
//     }
//
//   
//   return 0;
// } 
