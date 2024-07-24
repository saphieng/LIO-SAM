#include "utility.hpp"
#include <pcl/filters/voxel_grid.h>

const int queueLength = 2000;

class HighResolutionMapping : public ParamServer
{
private:
    std::mutex lasLock;
    std::mutex odoLock;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
    rclcpp::CallbackGroup::SharedPtr callbackGroupLidar;
    std::deque<sensor_msgs::msg::PointCloud2> cloudQueue;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
    rclcpp::CallbackGroup::SharedPtr callbackGroupOdom;
    std::deque<nav_msgs::msg::Odometry> odomQueue;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubFullCloud;

    sensor_msgs::msg::PointCloud2 currentCloudMsg;

    pcl::PointCloud<PointType>::Ptr tmpCloud;

    pcl::PointCloud<PointType>::Ptr fullCloud;
    pcl::PointCloud<PointType>::Ptr voxelCloud;

    // Voxel
    pcl::VoxelGrid<pcl::PointXYZI> voxelFilter;

public:
    HighResolutionMapping(const rclcpp::NodeOptions &options) : ParamServer("lio_sam_highResMapping", options)
    {
        callbackGroupLidar = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        callbackGroupOdom = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        auto lidarOpt = rclcpp::SubscriptionOptions();
        lidarOpt.callback_group = callbackGroupLidar;
        auto odomOpt = rclcpp::SubscriptionOptions();
        odomOpt.callback_group = callbackGroupOdom;

        subOdom = create_subscription<nav_msgs::msg::Odometry>(
            "lio_sam/mapping/odometry", qos_imu,
            std::bind(&HighResolutionMapping::odometryHandler, this, std::placeholders::_1),
            odomOpt);
        subLaserCloud = create_subscription<sensor_msgs::msg::PointCloud2>(
            "lio_sam/deskew/cloud_deskewed", qos_lidar,
            std::bind(&HighResolutionMapping::cloudHandler, this, std::placeholders::_1),
            lidarOpt);

        pubFullCloud = create_publisher<sensor_msgs::msg::PointCloud2>(
            "lio_sam/cloud_world", 1);

        allocateMemory();
        resetParameters();

        voxelFilter.setLeafSize(globalMapLeafSize, globalMapLeafSize, globalMapLeafSize);

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory()
    {
        tmpCloud.reset(new pcl::PointCloud<PointType>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        voxelCloud.reset(new pcl::PointCloud<PointType>());

        resetParameters();
    }

    void resetParameters()
    {
        tmpCloud->clear();
        fullCloud->clear();
        voxelCloud->clear();
    }

    ~HighResolutionMapping() {}

    void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
    {
        std::lock_guard<std::mutex> lock2(lasLock);
        cloudQueue.push_back(*laserCloudMsg);

        if (cloudQueue.size() <= 2)
            return;

        // convert cloud
        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();
        pcl::moveFromROSMsg(currentCloudMsg, *tmpCloud);

        double timeScanCur = stamp2Sec(currentCloudMsg.header.stamp);
        
        // Sync odometry
        while (!odomQueue.empty())
        {
            if (stamp2Sec(odomQueue.front().header.stamp) < timeScanCur - 0.01)
            {
                std::lock_guard<std::mutex> lock2(odoLock);
                odomQueue.pop_front();
            }
            else
                break;
        }

        if (odomQueue.empty())
            return;

        if (stamp2Sec(odomQueue.front().header.stamp) > timeScanCur)
            return;

        // get odometry at the beinning of the scan
        nav_msgs::msg::Odometry startOdomMsg;
        {
            std::lock_guard<std::mutex> lock2(odoLock);
            startOdomMsg = std::move(odomQueue.front());
        }

        // for (int i = 0; i < (int)odomQueue.size(); ++i)
        // {
        //     std::lock_guard<std::mutex> lock2(odoLock);
        //     startOdomMsg = odomQueue[i];

        //     if (stamp2Sec(startOdomMsg.header.stamp) < timeScanCur)
        //         continue;
        //     else
        //         break;
        // }

        // Extract rotation and translation from the odometry message
        if (startOdomMsg.pose.pose.orientation.x == 0.0 && startOdomMsg.pose.pose.orientation.y == 0.0 &&
            startOdomMsg.pose.pose.orientation.z == 0.0 && startOdomMsg.pose.pose.orientation.w == 0.0)
        {
            std::cerr << "Invalid orientation in odometry message" << std::endl;
            return;
        }

        tf2::Quaternion orientation;
        tf2::fromMsg(startOdomMsg.pose.pose.orientation, orientation);
        // std::cout << "Orientation: " << orientation.x() << ", " << orientation.y() << ", " << orientation.z() << ", " << orientation.w() << std::endl;


        tf2::Matrix3x3 mat(orientation);
        tf2::Vector3 translation(startOdomMsg.pose.pose.position.x,
                                 startOdomMsg.pose.pose.position.y,
                                 startOdomMsg.pose.pose.position.z);
        // std::cout << "Translation: " << translation.x() << ", " << translation.y() << ", " << translation.z() << std::endl;

        // Copy points to fullCloud
        try
        {
            for (int i = 0; i < tmpCloud->points.size(); i++)
            {
                pcl::PointXYZI tmpPoint;
                tmpPoint.x = tmpCloud->points[i].x;
                tmpPoint.y = tmpCloud->points[i].y;
                tmpPoint.z = tmpCloud->points[i].z;
                tmpPoint.intensity = tmpCloud->points[i].intensity;

                // Print out the point before transformation
                // std::cout << "Original Point: " << tmpPoint.x << ", " << tmpPoint.y << ", " << tmpPoint.z << std::endl;

                // Apply rotation and translation
                tf2::Vector3 point(tmpPoint.x, tmpPoint.y, tmpPoint.z);
                tf2::Vector3 transformedPoint = mat * point + translation;

                tmpPoint.x = transformedPoint.x();
                tmpPoint.y = transformedPoint.y();
                tmpPoint.z = transformedPoint.z();
                // Print out the point after transformation
                // std::cout << "Transformed Point: " << tmpPoint.x << ", " << tmpPoint.y << ", " << tmpPoint.z << std::endl;

                fullCloud->points.push_back(tmpPoint);
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Exception caught during point transformation: " << e.what() << std::endl;
            return;
        }

        // Voxelise the cloud
        voxelFilter.setInputCloud(fullCloud);
        voxelFilter.filter(*voxelCloud);

        // Set voxelCloud as the new fullCloud for the next cycle
        *fullCloud = *voxelCloud;

        publishCloud(pubFullCloud, fullCloud, currentCloudMsg.header.stamp, mapFrame);

    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor exec;

    auto HRM = std::make_shared<HighResolutionMapping>(options);
    exec.add_node(HRM);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> High Resolution Mapping Started.\033[0m");

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
