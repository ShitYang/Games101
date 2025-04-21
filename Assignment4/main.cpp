#include <chrono>
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 10) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm

    if (control_points.size() == 1)
    {
        return control_points.back();
    }

    std::vector<cv::Point2f> new_points(control_points.size() - 1);
    for (int i = 0; i < control_points.size() - 1; ++i)
    {
        new_points[i] = (1 - t) * control_points[i] + t * control_points[i + 1];
    }

    return recursive_bezier(new_points, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    float dx[4] = {-.5f, .5f, -.5f, .5f}, dy[4] = {-.5f, -.5f, .5f, .5f};
    for (float t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = recursive_bezier(control_points, t);

        float x = point.x, y = point.y;
        
        float center_x = std::floor(x), center_y = std::floor(y);
        center_x = (x - center_x) < .5f ? center_x : center_x + 1;
        center_y = (y - center_y) < .5f ? center_y : center_y + 1;

        for (int i = 0; i < 4; ++i)
        {
            cv::Point2f p(center_x + dx[i], center_y + dy[i]);
            cv::Point2f vector = p - point;

            float dist = std::sqrt(vector.dot(vector));
            float color = 255.f * (1 - dist / 1.414f);

            if (window.at<cv::Vec3b>(center_y + dy[i], center_x + dx[i])[1] < color)
            {
                window.at<cv::Vec3b>(center_y + dy[i], center_x + dx[i])[1] = color;
            }
        }

    }
}



int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 6) 
        {
            // naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

    return 0;
}
