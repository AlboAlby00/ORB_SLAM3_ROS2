#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

class MonocularInertialNode : public rclcpp::Node
{
public:
    MonocularInertialNode(ORB_SLAM3::System* pSLAM);
    ~MonocularInertialNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    using ImuMsg = sensor_msgs::msg::Imu;
    using PointMsg = geometry_msgs::msg::PointStamped;

    void GrabImage(const ImageMsg::SharedPtr msg);
    void GrabImu(const ImuMsg::SharedPtr msg);
    void SyncWithImu();

    std::thread *syncThread_;

    ORB_SLAM3::System* m_SLAM;

    cv_bridge::CvImagePtr m_cvImPtr;
    
    // IMU
    std::queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex imuMutex_;

    // Image
    std::queue<ImageMsg::SharedPtr> imageBuf_;
    std::mutex imageMutex_;

    rclcpp::Subscription<ImageMsg>::SharedPtr m_image_subscriber;
    rclcpp::Subscription<ImuMsg>::SharedPtr m_imu_subscriber;

    rclcpp::Publisher<PointMsg>::SharedPtr m_pose_publisher;

    double m_taw_low_pass;
    double m_previous_x, m_previous_y, m_previous_z;
    rclcpp::Time m_previous_time;

};

#endif
