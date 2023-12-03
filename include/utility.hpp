#ifndef __UTILITY_HPP__
#define __UTILITY_HPP__

#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/msg/image.hpp"

class Utility
{
  public:
      static double StampToSec(builtin_interfaces::msg::Time stamp)
      {
        double seconds = stamp.sec + (stamp.nanosec * pow(10,-9));
        return seconds;
      }
      static cv::Mat GetImage(const sensor_msgs::msg::Image::SharedPtr msg)
      {
            // Copy the ros image message to cv::Mat.
            cv_bridge::CvImageConstPtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
            }
            catch (cv_bridge::Exception &e)
            {
                std::cout << "cv_bridge exception: " << e.what() << std::endl;
            }

            if (cv_ptr->image.type() == 0)
            {
                return cv_ptr->image.clone();
            }
            else
            {
                std::cerr << "Error image type" << std::endl;
                return cv_ptr->image.clone();
            }
      }
};

#endif
