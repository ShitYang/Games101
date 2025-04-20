//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {

        if (u < 0) u = 0;
        if (u > 1) u = 1;
        if (v < 0) v = 0;
        if (v > 1) v = 1;

        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);

        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {

        if (u < 0) u = 0;
        if (u > 1) u = 1;
        if (v < 0) v = 0;
        if (v > 1) v = 1;

        float u_img = u * width, v_img = (1 - v) * height;
        int center_u = u_img, center_v = v_img;
        
        center_u = (u_img - center_u) < 0.5 ? center_u : center_u + 1;
        center_v = (v_img - center_v) < 0.5 ? center_v : center_v + 1;

        if (center_u <= 0 || center_v <= 0)
        {
            auto color = image_data.at<cv::Vec3b>(v_img, u_img);
            return Eigen::Vector3f(color[0], color[1], color[2]); 
        }

        auto pixel_color00 = image_data.at<cv::Vec3b>(center_v - 1, center_u - 1);
        auto pixel_color01 = image_data.at<cv::Vec3b>(center_v - 1, center_u);
        auto pixel_color10 = image_data.at<cv::Vec3b>(center_v, center_u - 1);
        auto pixel_color11 = image_data.at<cv::Vec3b>(center_v, center_u);
        
        Eigen::Vector3f color00(pixel_color00[0], pixel_color00[1], pixel_color00[2]);
        Eigen::Vector3f color01(pixel_color01[0], pixel_color01[1], pixel_color01[2]);
        Eigen::Vector3f color10(pixel_color10[0], pixel_color10[1], pixel_color10[2]);
        Eigen::Vector3f color11(pixel_color11[0], pixel_color11[1], pixel_color11[2]);

        float s = u_img - (center_u - .5f), t = v_img - (center_v - .5f);

        auto color_upper = color00 + s * (color01 - color00);
        auto color_down = color10 + s * (color11 - color10);

        return color_upper + t * (color_down - color_upper);
    }

};
#endif //RASTERIZER_TEXTURE_H
