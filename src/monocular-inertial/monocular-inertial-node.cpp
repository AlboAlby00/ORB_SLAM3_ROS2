#include "monocular-inertial-node.hpp"
#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularInertialNode::MonocularInertialNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2"), m_previous_x(0.0), m_previous_y(0.0), m_previous_z(0.0), m_taw_low_pass(1000)
{
    m_SLAM = pSLAM;

    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera",
        10,
        std::bind(&MonocularInertialNode::GrabImage, this, std::placeholders::_1));

    m_imu_subscriber = this->create_subscription<ImuMsg>(
        "imu",
        10,
        std::bind(&MonocularInertialNode::GrabImu, this, std::placeholders::_1));

    m_pose_publisher = this->create_publisher<PointMsg>("/crazyflie/camera_position", 10);
    
    syncThread_ = new std::thread(&MonocularInertialNode::SyncWithImu, this);

    m_previous_time = now();

}

MonocularInertialNode::~MonocularInertialNode()
{
    // Stop all threads
    m_SLAM->Shutdown();
}

void MonocularInertialNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    imageMutex_.lock();

    if (!imageBuf_.empty())
        imageBuf_.pop();
    imageBuf_.push(msg);

    imageMutex_.unlock();
}

void MonocularInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    imuMutex_.lock();
    imuBuf_.push(msg);
    imuMutex_.unlock();
}

void MonocularInertialNode::SyncWithImu()
{
    const double maxTimeDiff = 0.01;

    while(rclcpp::ok())
    {
        cv::Mat image;
        double tImage;
        double tImu;
        if (!imageBuf_ .empty() && !imuBuf_.empty())
        {
            tImage = Utility::StampToSec(imageBuf_.front()->header.stamp);
            tImu = Utility::StampToSec(imuBuf_.back()->header.stamp);

            if(tImage > tImu)
                continue;

            imageMutex_.lock();
            image = Utility::GetImage(imageBuf_.front());
            imageMutex_.unlock();

            std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
            imuMutex_.lock();
            if (!imuBuf_.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImage)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop();
                }
            }
            imuMutex_.unlock();

            Sophus::SE3f camera_pose = m_SLAM->TrackMonocular(image, tImage, vImuMeas);

            // Publish pose 
            
            PointMsg::SharedPtr pose_msg = std::make_shared<PointMsg>();

            rclcpp::Duration dt = now() - m_previous_time; 

            // filter
            double x = m_previous_x + (m_taw_low_pass / (dt.seconds() + m_taw_low_pass)) * (camera_pose.translation().x() - m_previous_x);
            double y = m_previous_y + (m_taw_low_pass / (dt.seconds() + m_taw_low_pass)) * (camera_pose.translation().y() - m_previous_y);
            double z = m_previous_z + (m_taw_low_pass / (dt.seconds() + m_taw_low_pass)) * (camera_pose.translation().z() - m_previous_z);
            m_previous_x = x;
            m_previous_y = y;
            m_previous_z = z;
            
            // this is for simulation
            pose_msg->point.x = - z;
            pose_msg->point.y = x;
            pose_msg->point.z = y;

            // this is for real world
            

            RCLCPP_DEBUG(this->get_logger(), "x: %f, y: %f, z: %f", pose_msg->point.x, pose_msg->point.y, pose_msg->point.z);

            m_pose_publisher->publish(*pose_msg);
            
            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
        
    }
}
