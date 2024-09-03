// Edited from https://github.com/JokerJohn/LIO_SAM_6AXIS/blob/d026151c12588821de8b7dd240b3ca7012da007d/LIO-SAM-6AXIS/src/simpleGpsOdom.cpp
// Mark Jin Edited 20230523
// Clint Jeffree Edited 20240822

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <mutex>
#include <queue>

#include "lio_sam/gpsTools.hpp"
#include <rclcpp/rclcpp.hpp>
#include "lio_sam/utility.hpp"

class GNSSOdom : public ParamServer
{
    public:
        GNSSOdom(const rclcpp::NodeOptions &options) : ParamServer("lio_sam_gnss_odom", options)
        {
            gpsSub = create_subscription<sensor_msgs::msg::NavSatFix>(
                gpsTopic, qos, std::bind(&GNSSOdom::GNSSCB, this, std::placeholders::_1));

            // imuSub = create_subscription<sensor_msgs::msg::Imu>(
            //     "/Imu", qos, std::bind(&GNSSOdom::IMUCB, this, std::placeholders::_1));

            // imuCorrectedPub = create_publisher<sensor_msgs::msg::Imu>("lio_sam/imu/imu_corrected", 100);
            gpsOdomPub = create_publisher<nav_msgs::msg::Odometry>("lio_sam/gps/odom", 100);
            gpsOriginPub = create_publisher<sensor_msgs::msg::NavSatFix>("lio_sam/gps/origin", 100);
            fusedPathPub = create_publisher<nav_msgs::msg::Path>("lio_sam/gps/path", 100);
        }

    private:
        void IMUCB(const sensor_msgs::msg::Imu::ConstSharedPtr &msg)
        {
            sensor_msgs::msg::Imu imu_msg = *msg;

            double imuRoll, imuPitch, imuYaw;
            tf2::Quaternion input;
            tf2::fromMsg(imu_msg.orientation, input);
            tf2::Matrix3x3(input).getRPY(imuRoll, imuPitch, imuYaw);

            // std::cout << "IN IMU - roll: " << imuRoll*180.0f/M_PI << ", pitch: " << imuPitch*180.0f/M_PI << ", yaw: " << imuYaw*180.0f/M_PI << std::endl;

            imuYaw += M_PI_2;
            // if (imuYaw < 0.0) {
            //     imuYaw += 2.0*M_PI;
            // }

            tf2::Quaternion output;
            output.setRPY(imuRoll, imuPitch, imuYaw);

            // std::cout << "OUT IMU - roll: " << imuRoll*180.0f/M_PI << ", pitch: " << imuPitch*180.0f/M_PI << ", yaw: " << imuYaw*180.0f/M_PI << std::endl;

            imu_msg.orientation.x = output.x();
            imu_msg.orientation.y = output.y();
            imu_msg.orientation.z = output.z();
            imu_msg.orientation.w = output.w();

            imuCorrectedPub->publish(imu_msg);

        }

        void GNSSCB(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &msg)
        {
            // gps status
            // std::cout << "gps status: " << msg->status.status << std::endl;
            if (std::isnan(msg->latitude + msg->longitude + msg->altitude))
            {
                RCLCPP_ERROR(this->get_logger(), "POS LLA NAN...");
                return;
            }

            double gps_time = static_cast<double>(msg->header.stamp.sec) + msg->header.stamp.nanosec / 10e9;
            Eigen::Vector3d lla(msg->latitude, msg->longitude, msg->altitude);

            // std::cout << "LLA: " << lla.transpose() << std::endl;
            if (!initXyz)
            {
                RCLCPP_INFO(this->get_logger(), "Init Orgin GPS LLA  %f, %f, %f", msg->latitude, msg->longitude,
                            msg->altitude);
                gtools.lla_origin_ = lla;

                originGps.latitude = lla(0);
                originGps.longitude = lla(1);
                originGps.altitude = lla(2);

                initXyz = true;
                return;
            }

            // RCLCPP_INFO_THROTTLE(
            //     this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
            //     "Init Orgin GPS Latitude, Longitude, Altitude  %f, %f, %f", gtools.lla_origin_[0], gtools.lla_origin_[1], gtools.lla_origin_[2]);

            // Convert LLA to ENU
            Eigen::Vector3d ecef = gtools.LLA2ECEF(lla);
            Eigen::Vector3d enu = gtools.ECEF2ENU(ecef);
            // RCLCPP_INFO(this->get_logger(), "GPS ENU: %f, %f, %f", enu(0), enu(1), enu(2));

            // Sometimes you may get a wrong origin at the beginning if the GPS signal is bad...
            if (abs(enu.x()) > 10000 || abs(enu.x()) > 10000 || abs(enu.x()) > 10000)
            {
                RCLCPP_INFO(this->get_logger(), "Error origin : %f, %f, %f", enu(0), enu(1), enu(2));

                ResetOrigin(lla);
                return;
            }

            if(prevPos.array().isNaN().any())
            {
                prevPos = enu;
            }

            // Additional extrincs between GNSS and IMU
            // most of the time, they are in the same frame
            Eigen::Matrix3d mat;
            mat <<  1, 0, 0,
                    0, 1, 0,
                    0, 0, 1;

            Eigen::Vector3d calib_enu = mat*enu;

            double distance = sqrt(pow(enu(1) - prevPos(1), 2) + pow(enu(0) - prevPos(0), 2));

            if (distance > 0.1)
            {
                // Calculate yaw using the position deltas
                yaw = atan2(enu(1) - prevPos(1), enu(0) - prevPos(0));

                if (yaw < 0.0) {
                    yaw += 2.0*M_PI;
                }
                
                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);

                yawQuat = tf2::toMsg(q);

                prevPos = enu;

                orientationReady_ = true;

                if (!firstYawInit)
                {
                    RCLCPP_INFO(this->get_logger(), "Init Yaw Success!");
                    firstYawInit = true;

                    ResetOrigin(lla);
                }
                RCLCPP_DEBUG(this->get_logger(), "GPS yaw: %f", yaw);

                // std::cout << "GPS YAW: " << yaw*180.0f/M_PI << std::endl;
            }
            else
            {
                orientationReady_ = false;
                return;
            }

            // Make sure your initial yaw and origin postion are consistent
            if (!firstYawInit || !orientationReady_)
            {
                RCLCPP_ERROR(this->get_logger(), "Waiting init origin yaw");
                return;
            }

            // Pub gps odometry
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = msg->header.stamp;
            odom_msg.header.frame_id = odometryFrame;
            odom_msg.child_frame_id = "gps";
            odom_msg.pose.pose.position.x = calib_enu(0);
            odom_msg.pose.pose.position.y = calib_enu(1);
            odom_msg.pose.pose.position.z = calib_enu(2);
            odom_msg.pose.covariance[0] = msg->position_covariance[0];
            odom_msg.pose.covariance[7] = msg->position_covariance[4];
            odom_msg.pose.covariance[14] = msg->position_covariance[8];
            odom_msg.pose.covariance[1] = lla[0];
            odom_msg.pose.covariance[2] = lla[1];
            odom_msg.pose.covariance[3] = lla[2];
            odom_msg.pose.covariance[4] = msg->status.status;
            odom_msg.pose.pose.orientation = yawQuat;
            gpsOdomPub->publish(odom_msg);

            // Publish path
            rospath.header.frame_id = odometryFrame;
            rospath.header.stamp = msg->header.stamp;
            geometry_msgs::msg::PoseStamped pose;
            pose.header = rospath.header;
            pose.pose.position.x = calib_enu(0);
            pose.pose.position.y = calib_enu(1);
            pose.pose.position.z = calib_enu(2);
            pose.pose.orientation.x = yawQuat.x;
            pose.pose.orientation.y = yawQuat.y;
            pose.pose.orientation.z = yawQuat.z;
            pose.pose.orientation.w = yawQuat.w;
            rospath.poses.push_back(pose);
            fusedPathPub->publish(rospath);

            gpsOriginPub->publish(originGps);
        }

    void ResetOrigin(Eigen::Vector3d &_lla) { gtools.lla_origin_ = _lla; }

    GpsTools gtools;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gpsOdomPub;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gpsOriginPub;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr fusedPathPub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuCorrectedPub;
    
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gpsSub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub;

    std::mutex mutexLock;
    std::deque<sensor_msgs::msg::NavSatFix::ConstSharedPtr> gpsBuf;

    sensor_msgs::msg::NavSatFix originGps;

    bool orientationReady_ = false;
    bool initXyz = false;
    bool firstYawInit = false;
    Eigen::Vector3d prevPos = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    double yaw = 0.0;
    geometry_msgs::msg::Quaternion yawQuat;
    nav_msgs::msg::Path rospath;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::SingleThreadedExecutor exec;

    auto GO = std::make_shared<GNSSOdom>(options);
    exec.add_node(GO);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Simple GPS Odmetry Started.\033[0m");

    exec.spin();

    rclcpp::shutdown();
    return 0;
}