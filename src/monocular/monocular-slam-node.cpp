#include "monocular-slam-node.hpp"
#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2") , m_previous_x(0.0), m_previous_y(0.0), m_previous_z(0.0), m_taw_low_pass(1000)
{
    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    std::cout << "slam changed" << std::endl;

    m_pose_publisher = this->create_publisher<PointMsg>("/crazyflie/camera_position_disaligned", 10);
    m_previous_time = now();
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();
}


void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    //std::cout<<"one frame has been sent"<<std::endl;
    Sophus::SE3f camera_pose = m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));

    PointMsg::SharedPtr pose_msg = std::make_shared<PointMsg>();

    rclcpp::Duration dt = now() - m_previous_time; 

    // filter
    double x = m_previous_x + (m_taw_low_pass / (dt.seconds() + m_taw_low_pass)) * (camera_pose.translation().x() - m_previous_x);
    double y = m_previous_y + (m_taw_low_pass / (dt.seconds() + m_taw_low_pass)) * (camera_pose.translation().y() - m_previous_y);
    double z = m_previous_z + (m_taw_low_pass / (dt.seconds() + m_taw_low_pass)) * (camera_pose.translation().z() - m_previous_z);
    m_previous_x = x;
    m_previous_y = y;
    m_previous_z = z;
    
    pose_msg->point.x = x;
    pose_msg->point.y = y;
    pose_msg->point.z = z;


    m_pose_publisher->publish(*pose_msg);

}
