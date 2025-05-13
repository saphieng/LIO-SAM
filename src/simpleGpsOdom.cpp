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
#include <std_srvs/srv/trigger.hpp>

#include <deque>
#include <mutex>
#include <queue>

#include "lio_sam/gpsTools.hpp"
#include "lio_sam/srv/set_gps_origin.hpp"
#include <rclcpp/rclcpp.hpp>
#include "lio_sam/utility.hpp"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class GNSSOdom : public ParamServer
{
    GpsTools gtools_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gpsOdomPub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gpsOriginPub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr fusedPathPub_;
    
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gpsSub_;

    rclcpp::Service<lio_sam::srv::SetGPSOrigin>::SharedPtr initOriginService_;
    // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetOriginService_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr initOriginSuccessService_;

    sensor_msgs::msg::NavSatFix originGps_;

    bool originInit_ = false;
    bool orientationReady_ = false;
    bool firstYawInit_ = false;

    Eigen::Vector3d prevPos_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    double yaw_ = 0.0;
    
    geometry_msgs::msg::Quaternion yawQuat_;
    nav_msgs::msg::Path rosPath_;

    public:
        GNSSOdom(const rclcpp::NodeOptions &options) : ParamServer("lio_sam_gnss_odom", options)
        {
            gpsSub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
                gpsTopic, qos, std::bind(&GNSSOdom::GNSSCB, this, std::placeholders::_1));

            gpsOdomPub_ = create_publisher<nav_msgs::msg::Odometry>("lio_sam/gps/odom", 100);
            gpsOriginPub_ = create_publisher<sensor_msgs::msg::NavSatFix>("lio_sam/gps/origin", 100);
            fusedPathPub_ = create_publisher<nav_msgs::msg::Path>("lio_sam/gps/path", 100);

            // Setup origin service
            initOriginService_ = create_service<lio_sam::srv::SetGPSOrigin>(
                "lio_sam/gps/origin/init",
                std::bind(&GNSSOdom::originInitCallback, this, std::placeholders::_1, std::placeholders::_2));
        }

    private:
        void originInitCallback(
            const std::shared_ptr<lio_sam::srv::SetGPSOrigin::Request> request,
            std::shared_ptr<lio_sam::srv::SetGPSOrigin::Response> response)
        {
            RCLCPP_INFO(this->get_logger(), "Setting GPS Origin...");

            json j = json::parse(request->message);

            std::cout << "Received JSON: " << j.dump(4) << std::endl;


            double lat = j["latitude"];
            double lon = j["longitude"];
            double alt = j["altitude"];

            Eigen::Vector3d lla(lat, lon, alt);

            ResetOrigin(lla);

            originGps_.latitude = lla(0);
            originGps_.longitude = lla(1);
            originGps_.altitude = lla(2);

            std::string response_msg = "Set GPS Origin: " + std::to_string(lat) + ", " + std::to_string(lon) + ", " + std::to_string(alt);
            
            originInit_ = true;
            
            response->success = originInit_;
            response->message = response_msg;

            RCLCPP_INFO(this->get_logger(), response_msg.c_str());

            // // Setup success service
            initOriginSuccessService_ = create_service<std_srvs::srv::Trigger>(
                "lio_sam/gps/origin/initSuccess",
                std::bind(&GNSSOdom::initOriginSuccessCallback, this, std::placeholders::_1, std::placeholders::_2));
        }
        
        void initOriginSuccessCallback(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            RCLCPP_INFO(this->get_logger(), "Origin initialised successfully.");

            response->success = true;
            response->message = "Origin initialised successfully.";
        }

        void GNSSCB(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &msg)
        {
            // gps status
            // std::cout << "gps status: " << msg->status.status << std::endl;
            if (std::isnan(msg->latitude + msg->longitude + msg->altitude))
            {
                RCLCPP_ERROR(this->get_logger(), "Latitude/Longitude/Altitude is NAN...");
                return;
            }

            double gps_time = static_cast<double>(msg->header.stamp.sec) + msg->header.stamp.nanosec / 10e9;
            Eigen::Vector3d lla(msg->latitude, msg->longitude, msg->altitude);

            // std::cout << "LLA: " << lla.transpose() << std::endl;
            if (!originInit_)
            {

                // RCLCPP_INFO_THROTTLE(
                // this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
                // "Waiting for initialise origin service...");

                if (!originServiceEnabled)
                {
                    RCLCPP_ERROR(this->get_logger(), "Origin service is not enabled...");

                    // Use origin from params file
                    if (abs(originLatitude) <= 90.0f && abs(originLatitude) <= 180.0f)
                    {
                        lla[0] = originLatitude;
                        lla[1] = originLongitude;
                        lla[2] = originAltitude;
                    }

                    RCLCPP_INFO(this->get_logger(), "Setting GPS Origin from config file: %f, %f, %f", lla(0), lla(1), lla(2));  

                    ResetOrigin(lla);

                    originGps_.latitude = lla(0);
                    originGps_.longitude = lla(1);
                    originGps_.altitude = lla(2);

                    originInit_ = true;
                }
                return;
            }

            // Convert LLA to ENU
            Eigen::Vector3d ecef = gtools_.LLA2ECEF(lla);
            Eigen::Vector3d enu = gtools_.ECEF2ENU(ecef);
            // RCLCPP_INFO(this->get_logger(), "GPS ENU: %f, %f, %f", enu(0), enu(1), enu(2));

            // Sometimes you may get a wrong origin at the beginning if the GPS signal is bad...
            if (abs(enu.x()) > 10000 || abs(enu.x()) > 10000 || abs(enu.x()) > 10000)
            {
                RCLCPP_INFO(this->get_logger(), "Error origin : %f, %f, %f", enu(0), enu(1), enu(2));

                originInit_ = false;
                // ResetOrigin(lla);
                return;
            }

            if(prevPos_.array().isNaN().any())
            {
                prevPos_ = enu;
            }

            // Additional extrincs between GNSS and IMU
            // most of the time, they are in the same frame
            Eigen::Matrix3d mat;
            mat <<  1, 0, 0,
                    0, 1, 0,
                    0, 0, 1;

            Eigen::Vector3d calib_enu = mat*enu;

            double distance = sqrt(pow(enu(1) - prevPos_(1), 2) + pow(enu(0) - prevPos_(0), 2));

            if (distance > 0.1)
            {
                // Calculate yaw_ using the position deltas
                yaw_ = atan2(enu(1) - prevPos_(1), enu(0) - prevPos_(0));

                if (yaw_ < 0.0) {
                    yaw_ += 2.0*M_PI;
                }
                
                tf2::Quaternion q;
                q.setRPY(0, 0, yaw_);

                yawQuat_ = tf2::toMsg(q);

                if (!firstYawInit_)
                {
                    RCLCPP_INFO(this->get_logger(), "Init Yaw Success!");
                    firstYawInit_ = true;
                    
                    // ResetOrigin(lla);
                }
                
                RCLCPP_DEBUG(this->get_logger(), "GPS yaw: %f", yaw_);
                
                prevPos_ = enu;
                orientationReady_ = true;
            }
            else
            {
                orientationReady_ = false;
                return;
            }

            // Make sure your initial yaw_ and origin postion are consistent
            if (!firstYawInit_ || !orientationReady_)
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
            odom_msg.pose.pose.orientation = yawQuat_;
            gpsOdomPub_->publish(odom_msg);

            // Publish path
            rosPath_.header.frame_id = odometryFrame;
            rosPath_.header.stamp = msg->header.stamp;
            geometry_msgs::msg::PoseStamped pose;
            pose.header = rosPath_.header;
            pose.pose.position.x = calib_enu(0);
            pose.pose.position.y = calib_enu(1);
            pose.pose.position.z = calib_enu(2);
            pose.pose.orientation.x = yawQuat_.x;
            pose.pose.orientation.y = yawQuat_.y;
            pose.pose.orientation.z = yawQuat_.z;
            pose.pose.orientation.w = yawQuat_.w;
            rosPath_.poses.push_back(pose);
            fusedPathPub_->publish(rosPath_);

            gpsOriginPub_->publish(originGps_);
        }

    void ResetOrigin(Eigen::Vector3d &_lla) { gtools_.lla_origin_ = _lla; }
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