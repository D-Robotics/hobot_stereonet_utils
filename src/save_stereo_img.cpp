#include <sys/stat.h>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

class ImgReceiverNode : public rclcpp::Node
{
public:
    ImgReceiverNode() : Node("img_receiver_node")
    {
        this->declare_parameter("save_num", 1);
        this->get_parameter("save_num", save_num_);
        dir_ = this->declare_parameter("dir", dir_);
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(sub_topic_, 10, std::bind(&ImgReceiverNode::image_callback, this, std::placeholders::_1));
        RCLCPP_WARN(this->get_logger(), "Stereo sensor image collection node started! sub_topic: %s, save_num: %d, saved dir: %s", sub_topic_.data(), save_num_, dir_.data());
        if (access(dir_.c_str(), W_OK))
        {
            RCLCPP_ERROR(this->get_logger(), "saved dir [%s] is not existed", dir_.c_str());
            rclcpp::shutdown();
            return;
        }
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "\033[31m=> Receive combine rgb img\033[0m");

        // check whether the image encoding is NV12
        if (msg->encoding != "nv12")
        {
            RCLCPP_ERROR(this->get_logger(), "=> Unrecognized image encoding [%s]", msg->encoding.c_str());
            return;
        }

        // get the width and height of the image
        int width = msg->width;
        int height = msg->height;

        // create a Mat object in NV12 format
        cv::Mat nv12_image(height + height / 2, width, CV_8UC1, const_cast<unsigned char *>(msg->data.data()));

        // convert nv12 image to bgr format
        cv::Mat bgr_image;
        cv::cvtColor(nv12_image, bgr_image, cv::COLOR_YUV2BGR_NV12);

        // detect the keyboard and press the enter key
        RCLCPP_INFO_ONCE(this->get_logger(), "\033[92m=> Press Enter to save the image...\033[0m");
        if (std::cin.get() == '\n')
        {
            std::stringstream ss;
            ss << dir_ << "/combine_" << std::setw(3) << std::setfill('0') << save_num_ << ".png";
            cv::imwrite(ss.str(), bgr_image);
            RCLCPP_INFO(this->get_logger(), "\033[92m=> Image saved as [%s]\033[0m", ss.str().c_str());
            save_num_++;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    int save_num_ = 1;
    std::string dir_ = "./data/calib_imgs";
    std::string sub_topic_ = "/image_combine_raw";
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImgReceiverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}