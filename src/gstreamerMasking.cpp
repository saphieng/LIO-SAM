#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "utility.hpp"

class GStreamerMaskingNode : public ParamServer
{
public:
    GStreamerMaskingNode(const rclcpp::NodeOptions & options) : ParamServer("lio_sam_gstreamerMasking", options)
    {
        mask_pub_ = create_publisher<sensor_msgs::msg::Image>(maskTopic, 10);
        ouster_mask_pub_ = create_publisher<sensor_msgs::msg::Image>(imgMaskTopic, 10);

        std::cout << "Homography: " << homography << std::endl;
        std::cout << "Masking threshold: " << maskingThreshold << std::endl;
        std::cout << "Pixel buffer: " << pixelBuffer << std::endl;

        RCLCPP_INFO(this->get_logger(), "Launching GStreamer pipeline...");
        init_gstreamer_pipeline();
    }

    ~GStreamerMaskingNode()
    {
        if (pipeline_) gst_object_unref(pipeline_);
    }

private:
    void init_gstreamer_pipeline()
    {
        gst_init(nullptr, nullptr);

        std::string pipeline_desc =
            "tcpclientsrc host=127.0.0.1 port=5003 ! tsdemux ! h265parse ! nvv4l2decoder ! nvvidconv ! video/x-raw,format=GRAY8 ! appsink name=sink";

        GError *error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_desc.c_str(), &error);
        if (!pipeline_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create pipeline: %s", error->message);
            g_error_free(error);
            return;
        }

        appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
        g_object_set(appsink_, "emit-signals", TRUE, "sync", FALSE, nullptr);
        g_signal_connect(appsink_, "new-sample", G_CALLBACK(on_new_sample), this);

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);

        RCLCPP_INFO(this->get_logger(), "GStreamer pipeline launched...");
    }

    static GstFlowReturn on_new_sample(GstAppSink *sink, gpointer user_data)
    {
        auto *self = static_cast<GStreamerMaskingNode*>(user_data);
        GstSample *sample = gst_app_sink_pull_sample(sink);
        if (!sample) return GST_FLOW_ERROR;

        GstBuffer *buffer = gst_sample_get_buffer(sample);
        GstCaps *caps = gst_sample_get_caps(sample);
        GstStructure *s = gst_caps_get_structure(caps, 0);

        int width, height;
        gst_structure_get_int(s, "width", &width);
        gst_structure_get_int(s, "height", &height);

        GstMapInfo map;
        if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }

        cv::Mat mask_output(height, width, CV_8UC1, (void *)map.data, width);
        cv::Mat resized_image, transformed_image, dilated_image, ouster_mask_output;

        cv::resize(mask_output, resized_image, cv::Size(640, 640), 0, 0, cv::INTER_LINEAR);

        if (self->homography.type() != CV_64FC1 || self->homography.rows != 3 || self->homography.cols != 3) {
            std::cerr << "Homography has incorrect type or shape: "
                      << self->homography.type() << " (" 
                      << self->homography.rows << "x"
                      << self->homography.cols << ")" << std::endl;
            return GST_FLOW_ERROR;
        }

        // Transform and process
        cv::warpPerspective(resized_image, transformed_image, self->homography,
                            cv::Size(self->Horizon_SCAN, self->N_SCAN), cv::INTER_LINEAR,
                            cv::BORDER_CONSTANT, cv::Scalar(255));

        cv::bitwise_not(transformed_image, transformed_image);

        cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_RECT,
            cv::Size(2 * self->pixelBuffer + 1, 2 * self->pixelBuffer + 1));

        cv::dilate(transformed_image, dilated_image, kernel);

        cv::bitwise_not(dilated_image, dilated_image);

        cv::threshold(dilated_image, ouster_mask_output,
                      self->maskingThreshold, 255, cv::THRESH_BINARY);

        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);

        std_msgs::msg::Header header;
        header.stamp = self->now();
        header.frame_id = "camera";

        auto mask_msg = cv_bridge::CvImage(header, "mono8", resized_image).toImageMsg();
        auto ouster_msg = cv_bridge::CvImage(header, "mono8", ouster_mask_output).toImageMsg();

        self->mask_pub_->publish(*mask_msg);
        self->ouster_mask_pub_->publish(*ouster_msg);

        return GST_FLOW_OK;
    }

    // ROS
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ouster_mask_pub_;

    // GStreamer
    GstElement *pipeline_ = nullptr;
    GstElement *appsink_ = nullptr;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::SingleThreadedExecutor exec;

    auto GM = std::make_shared<GStreamerMaskingNode>(options);
    exec.add_node(GM);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> GStreamer Masking Started.\033[0m");

    exec.spin();

    rclcpp::shutdown();

    return 0;
}
