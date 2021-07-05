#include <rclcpp/rclcpp.hpp>
//#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

using std::placeholders::_1;

int initial_flag = 0;

std::string window_name = "image";

class realsens_subscriber:public rclcpp::Node
{
    public:
        realsens_subscriber():Node("realsense_subscriber")
        {
            cascade.load("./haarcascade_fullbody.xml");
            sub_ = this->create_subscription<sensor_msgs::msg::Image>("camera/color/image_raw", 1, std::bind(&realsens_subscriber::realsense_callback, this, _1));
            cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
        }
        ~realsens_subscriber()
        {
            cv::destroyAllWindows();
        }
    private:
        cv::CascadeClassifier cascade;
        std::vector<cv::Rect> contour;

        void realsense_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

void realsens_subscriber::realsense_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    auto data = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat gray;
    cv::cvtColor(data->image, gray, cv::COLOR_BGR2GRAY);
    cascade.detectMultiScale(data->image, contour, 1.1, 3, 0, cv::Size(30, 30));

    for(int i = 0; i < contour.size(); i++){
		cv::rectangle(data->image, cv::Point(contour[i].x, contour[i].y), cv::Point(contour[i].x + contour[i].width, contour[i].y + contour[i].height), cv::Scalar(255, 0, 0), 1);
	}

    cv::imshow(window_name.c_str(),data->image);
    cv::waitKey(1);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<realsens_subscriber>());
    rclcpp::shutdown();
    return 0;
}