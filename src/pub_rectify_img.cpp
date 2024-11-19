// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include <arm_neon.h>

class PubRectifyImgsNode : public rclcpp::Node
{
public:
    PubRectifyImgsNode() : Node("pub_rectify_imgs_node")
    {
        RCLCPP_INFO(this->get_logger(), "=> init pub_rectify_imgs_node!");
        // ======================================================================================================================================
        // param
        this->declare_parameter("calib_file_path", "");
        this->declare_parameter("only_pub_left", false);
        this->declare_parameter("pub_image_width", 1280);
        this->declare_parameter("pub_image_height", 640);
        this->get_parameter("calib_file_path", calib_file_path_);
        this->get_parameter("only_pub_left", only_pub_left_);
        this->get_parameter("pub_image_width", pub_image_width_);
        this->get_parameter("pub_image_height", pub_image_height_);
        RCLCPP_INFO_STREAM(this->get_logger(), "\033[31m=> calib_file_path: " << calib_file_path_ << std::endl
                                                                              << "=> only_pub_left: " << only_pub_left_ << std::endl
                                                                              << "=> pub_image_width: " << pub_image_width_ << std::endl
                                                                              << "=> pub_image_height: " << pub_image_height_ << std::endl
                                                                              << "\033[0m");
        // ======================================================================================================================================
        // pub & sub
        stereo_msg_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/image_combine_raw", 10, std::bind(&PubRectifyImgsNode::image_callback, this, std::placeholders::_1));
        stereo_msg_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_combine_rectify", 10);

        // ======================================================================================================================================
        cv::FileStorage fs(calib_file_path_, cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to open " << calib_file_path_);
            rclcpp::shutdown();
            return;
        }

        load_calib_file(fs["stereo0"], pub_image_width_, pub_image_height_);

        fs.release();
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "\033[31m=> receive stereo img: [" << msg->encoding << ", " << msg->width << ", " << msg->height << "]\033[0m");
        cv::Mat bgr(msg->height, msg->width, CV_8UC3);
        nv12_to_bgr24_neon(msg->data.data(), bgr.data, msg->width, msg->height);

        int stereo_img_width = msg->width, stereo_img_height = msg->height / 2;
        cv::Mat left_img = bgr(cv::Rect(0, 0, stereo_img_width, stereo_img_height)).clone();
        cv::Mat right_img = bgr(cv::Rect(0, stereo_img_height, stereo_img_width, stereo_img_height)).clone();

        cv::Mat rectified_left_img, rectified_right_img;
        rectify(left_img, right_img, rectified_left_img, rectified_right_img);
        if (only_pub_left_)
        {
            // create ros msg
            cv::Mat rectified_left_img_nv12;
            bgr_to_nv12(rectified_left_img, rectified_left_img_nv12);
            auto left_msg = std::make_shared<sensor_msgs::msg::Image>();
            left_msg->header = msg->header;
            left_msg->height = rectified_left_img.rows;
            left_msg->width = rectified_left_img.cols;
            left_msg->encoding = "nv12";
            left_msg->is_bigendian = false;
            left_msg->step = rectified_left_img.cols;
            left_msg->data.assign(rectified_left_img_nv12.data, rectified_left_img_nv12.data + (rectified_left_img_nv12.rows * rectified_left_img_nv12.cols));
            stereo_msg_pub_->publish(*left_msg);
        }
        else
        {
            cv::Mat frameBGR, combine_nv12;
            cv::vconcat(rectified_left_img, rectified_right_img, frameBGR);
            bgr_to_nv12(frameBGR, combine_nv12);

            // create ros msg
            auto stereo_msg = std::make_shared<sensor_msgs::msg::Image>();
            stereo_msg->header.stamp = this->now();
            stereo_msg->header.frame_id = "pcl_link";
            stereo_msg->height = frameBGR.rows;
            stereo_msg->width = frameBGR.cols;
            stereo_msg->encoding = "nv12";
            stereo_msg->is_bigendian = false;
            stereo_msg->step = frameBGR.cols;
            stereo_msg->data.assign(combine_nv12.data, combine_nv12.data + (combine_nv12.rows * combine_nv12.cols));
            stereo_msg_pub_->publish(*stereo_msg);
        }
    }

private:
    void load_calib_file(const cv::FileNode &fs, int model_input_w, int model_input_h)
    {
        float width_scale, height_scale;
        // Reading cam0 data
        std::vector<double> cam0_distortion_coeffs;
        std::vector<double> cam0_intrinsics;
        std::vector<int> cam0_resolution;

        fs["cam0"]["distortion_coeffs"] >> cam0_distortion_coeffs;
        fs["cam0"]["intrinsics"] >> cam0_intrinsics;
        fs["cam0"]["resolution"] >> cam0_resolution;

        width_scale = model_input_w / static_cast<float>(cam0_resolution[0]);
        height_scale = model_input_h / static_cast<float>(cam0_resolution[1]);

        // Reading cam1 data
        std::vector<std::vector<double>> cam1_T_cn_cnm1;
        std::vector<double> cam1_distortion_coeffs;
        std::vector<double> cam1_intrinsics;
        std::vector<int> cam1_resolution;

        fs["cam1"]["T_cn_cnm1"] >> cam1_T_cn_cnm1;
        fs["cam1"]["distortion_coeffs"] >> cam1_distortion_coeffs;
        fs["cam1"]["intrinsics"] >> cam1_intrinsics;
        fs["cam1"]["resolution"] >> cam1_resolution;

        Dl = cv::Mat(1, cam0_distortion_coeffs.size(), CV_64F, cam0_distortion_coeffs.data()).clone();
        Kl = cv::Mat::zeros(3, 3, CV_64F);
        Kl.at<double>(0, 0) = cam0_intrinsics[0] * width_scale;
        Kl.at<double>(0, 2) = cam0_intrinsics[2] * width_scale;
        Kl.at<double>(1, 1) = cam0_intrinsics[1] * height_scale;
        Kl.at<double>(1, 2) = cam0_intrinsics[3] * height_scale;
        Kl.at<double>(2, 2) = 1;

        R_rl = cv::Mat::zeros(3, 3, CV_64F);
        t_rl = cv::Mat::zeros(3, 1, CV_64F);

        R_rl.at<double>(0, 0) = cam1_T_cn_cnm1[0][0];
        R_rl.at<double>(0, 1) = cam1_T_cn_cnm1[0][1];
        R_rl.at<double>(0, 2) = cam1_T_cn_cnm1[0][2];
        R_rl.at<double>(1, 0) = cam1_T_cn_cnm1[1][0];
        R_rl.at<double>(1, 1) = cam1_T_cn_cnm1[1][1];
        R_rl.at<double>(1, 2) = cam1_T_cn_cnm1[1][2];
        R_rl.at<double>(2, 0) = cam1_T_cn_cnm1[2][0];
        R_rl.at<double>(2, 1) = cam1_T_cn_cnm1[2][1];
        R_rl.at<double>(2, 2) = cam1_T_cn_cnm1[2][2];

        t_rl.at<double>(0, 0) = cam1_T_cn_cnm1[0][3];
        t_rl.at<double>(1, 0) = cam1_T_cn_cnm1[1][3];
        t_rl.at<double>(2, 0) = cam1_T_cn_cnm1[2][3];

        Dr = cv::Mat(1, cam1_distortion_coeffs.size(), CV_64F, cam1_distortion_coeffs.data()).clone();

        Kr = cv::Mat::zeros(3, 3, CV_64F);
        Kr.at<double>(0, 0) = cam1_intrinsics[0] * width_scale;
        Kr.at<double>(0, 2) = cam1_intrinsics[2] * width_scale;
        Kr.at<double>(1, 1) = cam1_intrinsics[1] * height_scale;
        Kr.at<double>(1, 2) = cam1_intrinsics[3] * height_scale;
        Kr.at<double>(2, 2) = 1;

        cv::stereoRectify(Kl, Dl, Kr, Dr, cv::Size(model_input_w, model_input_h), R_rl, t_rl, Rl, Rr, Pl, Pr, Q, cv::CALIB_ZERO_DISPARITY, 0);

        cv::initUndistortRectifyMap(Kl, Dl, Rl, Pl, cv::Size(model_input_w, model_input_h), CV_32FC1, undistmap1l, undistmap2l);
        cv::initUndistortRectifyMap(Kr, Dr, Rr, Pr, cv::Size(model_input_w, model_input_h), CV_32FC1, undistmap1r, undistmap2r);

        camera_fx = Q.at<double>(2, 3);
        camera_fy = Q.at<double>(2, 3);
        camera_cx = -Q.at<double>(0, 3);
        camera_cy = -Q.at<double>(1, 3);
        base_line = std::abs(1 / Q.at<double>(3, 2));

        std::cout << "Kl:" << std::endl
                  << Kl << std::endl
                  << "Dl:" << std::endl
                  << Dl << std::endl
                  << "Kr: " << std::endl
                  << Kr << std::endl
                  << "Dr:" << std::endl
                  << Dr << std::endl
                  << "R, t: " << std::endl
                  << R_rl << std::endl
                  << t_rl << std::endl
                  << "calib file width, height: " << cam0_resolution[0] << ", " << cam0_resolution[1] << std::endl
                  << "width_scale, height_scale: " << width_scale << ", " << height_scale << std::endl
                  << "rectify [f, cx, cy, baseline]: " << "[" << camera_fx << ", " << camera_cx << ", " << camera_cy << ", " << base_line << "]" << std::endl
                  << std::endl;
    }

    void rectify(const cv::Mat &left_image, const cv::Mat &right_image, cv::Mat &rectified_left_image, cv::Mat &rectified_right_image)
    {
        cv::remap(left_image, rectified_left_image, undistmap1l, undistmap2l, cv::INTER_LINEAR);
        cv::remap(right_image, rectified_right_image, undistmap1r, undistmap2r, cv::INTER_LINEAR);
    }

    void bgr24_to_nv12_neon(uint8_t *bgr24, uint8_t *nv12, int width, int height)
    {
        int frameSize = width * height;
        int yIndex = 0;
        int uvIndex = frameSize;
        const uint16x8_t u16_rounding = vdupq_n_u16(128);
        const int16x8_t s16_rounding = vdupq_n_s16(128);
        const int8x8_t s8_rounding = vdup_n_s8(128);
        const uint8x16_t offset = vdupq_n_u8(16);
        const uint16x8_t mask = vdupq_n_u16(255);

        for (int j = 0; j < height; j++)
        {
            for (int i = 0; i < width >> 4; i++)
            {
                // Load rgb
                uint8x16x3_t pixel_rgb;
                pixel_rgb = vld3q_u8(bgr24);
                bgr24 += 48;

                uint8x8x2_t uint8_r;
                uint8x8x2_t uint8_g;
                uint8x8x2_t uint8_b;
                uint8_r.val[0] = vget_low_u8(pixel_rgb.val[2]);
                uint8_r.val[1] = vget_high_u8(pixel_rgb.val[2]);
                uint8_g.val[0] = vget_low_u8(pixel_rgb.val[1]);
                uint8_g.val[1] = vget_high_u8(pixel_rgb.val[1]);
                uint8_b.val[0] = vget_low_u8(pixel_rgb.val[0]);
                uint8_b.val[1] = vget_high_u8(pixel_rgb.val[0]);

                uint16x8x2_t uint16_y;
                uint8x8_t scalar = vdup_n_u8(66);
                uint8x16_t y;

                uint16_y.val[0] = vmull_u8(uint8_r.val[0], scalar);
                uint16_y.val[1] = vmull_u8(uint8_r.val[1], scalar);
                scalar = vdup_n_u8(129);
                uint16_y.val[0] = vmlal_u8(uint16_y.val[0], uint8_g.val[0], scalar);
                uint16_y.val[1] = vmlal_u8(uint16_y.val[1], uint8_g.val[1], scalar);
                scalar = vdup_n_u8(25);
                uint16_y.val[0] = vmlal_u8(uint16_y.val[0], uint8_b.val[0], scalar);
                uint16_y.val[1] = vmlal_u8(uint16_y.val[1], uint8_b.val[1], scalar);

                uint16_y.val[0] = vaddq_u16(uint16_y.val[0], u16_rounding);
                uint16_y.val[1] = vaddq_u16(uint16_y.val[1], u16_rounding);

                y = vcombine_u8(vqshrn_n_u16(uint16_y.val[0], 8), vqshrn_n_u16(uint16_y.val[1], 8));
                y = vaddq_u8(y, offset);

                vst1q_u8(nv12 + yIndex, y);
                yIndex += 16;

                // Compute u and v in the even row
                if (j % 2 == 0)
                {
                    int16x8_t u_scalar = vdupq_n_s16(-38);
                    int16x8_t v_scalar = vdupq_n_s16(112);

                    int16x8_t r = vreinterpretq_s16_u16(vandq_u16(vreinterpretq_u16_u8(pixel_rgb.val[2]), mask));
                    int16x8_t g = vreinterpretq_s16_u16(vandq_u16(vreinterpretq_u16_u8(pixel_rgb.val[1]), mask));
                    int16x8_t b = vreinterpretq_s16_u16(vandq_u16(vreinterpretq_u16_u8(pixel_rgb.val[0]), mask));

                    int16x8_t u;
                    int16x8_t v;
                    uint8x8x2_t uv;

                    u = vmulq_s16(r, u_scalar);
                    v = vmulq_s16(r, v_scalar);

                    u_scalar = vdupq_n_s16(-74);
                    v_scalar = vdupq_n_s16(-94);
                    u = vmlaq_s16(u, g, u_scalar);
                    v = vmlaq_s16(v, g, v_scalar);

                    u_scalar = vdupq_n_s16(112);
                    v_scalar = vdupq_n_s16(-18);
                    u = vmlaq_s16(u, b, u_scalar);
                    v = vmlaq_s16(v, b, v_scalar);

                    u = vaddq_s16(u, s16_rounding);
                    v = vaddq_s16(v, s16_rounding);

                    uv.val[0] = vreinterpret_u8_s8(vadd_s8(vqshrn_n_s16(u, 8), s8_rounding));
                    uv.val[1] = vreinterpret_u8_s8(vadd_s8(vqshrn_n_s16(v, 8), s8_rounding));

                    vst2_u8(nv12 + uvIndex, uv);

                    uvIndex += 16;
                }
            }
        }
    }

    void nv12_to_bgr24_neon(uint8_t *nv12, uint8_t *bgr24, int width, int height)
    {
        const uint8_t *yptr = nv12;
        const uint8_t *uvptr = nv12 + width * height;
        uint8x8_t _v128 = vdup_n_u8(128);
        int8x8_t _v127 = vdup_n_s8(127);
        uint8x8_t _v16 = vdup_n_u8(16);
        uint8x8_t _v75 = vdup_n_u8(75);
        uint8x8_t _vu64 = vdup_n_u8(64);
        int8x8_t _v52 = vdup_n_s8(52);
        int8x8_t _v25 = vdup_n_s8(25);
        int8x8_t _v102 = vdup_n_s8(102);
        int16x8_t _v64 = vdupq_n_s16(64);

        for (int y = 0; y < height; y += 2)
        {
            const uint8_t *yptr0 = yptr;
            const uint8_t *yptr1 = yptr + width;
            unsigned char *rgb0 = bgr24;
            unsigned char *rgb1 = bgr24 + width * 3;
            int nn = width >> 3;

            for (; nn > 0; nn--)
            {
                int16x8_t _yy0 = vreinterpretq_s16_u16(vmull_u8(vqsub_u8(vld1_u8(yptr0), _v16), _v75));
                int16x8_t _yy1 = vreinterpretq_s16_u16(vmull_u8(vqsub_u8(vld1_u8(yptr1), _v16), _v75));
                //      int16x8_t _yy0 = vreinterpretq_s16_u16(vmull_u8(vld1_u8(yptr0), _v75));
                //      int16x8_t _yy1 = vreinterpretq_s16_u16(vmull_u8(vld1_u8(yptr1), _v75));
                int8x8_t _uuvv = vreinterpret_s8_u8(vsub_u8(vld1_u8(uvptr), _v128));
                int8x8x2_t _uuuuvvvv = vtrn_s8(_uuvv, _uuvv);
                int8x8_t _uu = _uuuuvvvv.val[0];
                int8x8_t _vv = _uuuuvvvv.val[1];

                int16x8_t _r0 = vmlal_s8(_yy0, _vv, _v102);
                int16x8_t _g0 = vmlsl_s8(_yy0, _vv, _v52);
                _g0 = vmlsl_s8(_g0, _uu, _v25);
                int16x8_t _b0 = vmlal_s8(_yy0, _uu, _v127);

                int16x8_t _r1 = vmlal_s8(_yy1, _vv, _v102);
                int16x8_t _g1 = vmlsl_s8(_yy1, _vv, _v52);
                _g1 = vmlsl_s8(_g1, _uu, _v25);
                int16x8_t _b1 = vmlal_s8(_yy1, _uu, _v127);

                uint8x8x3_t _rgb0;
                _rgb0.val[2] = vqshrun_n_s16(vaddq_s16(_r0, _v64), 6);
                _rgb0.val[1] = vqshrun_n_s16(vaddq_s16(_g0, _v64), 6);
                _rgb0.val[0] = vqshrun_n_s16(vaddq_s16(_b0, _v64), 6);

                uint8x8x3_t _rgb1;
                _rgb1.val[2] = vqshrun_n_s16(vaddq_s16(_r1, _v64), 6);
                _rgb1.val[1] = vqshrun_n_s16(vaddq_s16(_g1, _v64), 6);
                _rgb1.val[0] = vqshrun_n_s16(vaddq_s16(_b1, _v64), 6);

                vst3_u8(rgb0, _rgb0);
                vst3_u8(rgb1, _rgb1);

                yptr0 += 8;
                yptr1 += 8;
                uvptr += 8;
                rgb0 += 24;
                rgb1 += 24;
            }
            yptr += 2 * width;
            bgr24 += 2 * 3 * width;
        }
    }

    void bgr_to_nv12(const cv::Mat &bgr, cv::Mat &nv12)
    {
        int width = bgr.cols;
        int height = bgr.rows;
        nv12 = cv::Mat(height * 3 / 2, width, CV_8UC1);
        bgr24_to_nv12_neon(bgr.data, nv12.data, width, height);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr stereo_msg_sub_; // stereo image subscription
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereo_msg_pub_;    // stereo image publisher

    // node param
    bool only_pub_left_;

    // cam param
    cv::Mat Rl, Rr, Pl, Pr, Q;
    cv::Mat Kl, Kr, Dl, Dr, R_rl, t_rl;
    cv::Mat undistmap1l, undistmap2l, undistmap1r, undistmap2r;
    float camera_cx, camera_cy, camera_fx, camera_fy, base_line;
    std::string calib_file_path_;
    int pub_image_width_;
    int pub_image_height_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PubRectifyImgsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}